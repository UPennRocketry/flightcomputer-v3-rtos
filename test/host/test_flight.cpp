// Host-side tests for the hardware-independent flight modules.
//
// Build & run (from repo root):
//   make -C test/host
//
// What is covered:
//   1. Telemetry packets: encode with the real C++ code, dump bytes for the
//      Python decoder (ground/telemetry_protocol.py) to cross-check.
//   2. A simulated flight through flight_logic::update at 100 Hz:
//      pad -> boost -> coast -> apogee -> deploy at <= kDeployAltM -> touchdown.
//   3. The same flight interrupted by a simulated watchdog reset during coast,
//      restored from the persisted record, and flown to deployment.
//   4. Backup-timer and pyro shut-off paths via the supervisor.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "Arduino.h"
#include "EEPROM.h"
#include "app_config.h"
#include "app_types.h"
#include "flight_logic.h"
#include "persist.h"
#include "rtos_context.h"
#include "sensors.h"
#include "state_estimator.h"
#include "supervisor.h"
#include "telemetry.h"

// ---- host globals the firmware expects ----
uint32_t g_host_millis = 0;
int g_host_pin_state[64] = {0};
HostSerial Serial;
HostEEPROM EEPROM;
uint32_t g_host_SRC_SRSR = 0;
uint16_t g_host_WDOG1_WCR = 0;
uint16_t g_host_WDOG1_WSR = 0;

QueueHandle_t g_flightQueue = nullptr;
QueueHandle_t g_logQueue = nullptr;
QueueHandle_t g_debugQueue = nullptr;
SemaphoreHandle_t g_stateMutex = reinterpret_cast<SemaphoreHandle_t>(1);
SemaphoreHandle_t g_spi0Mutex = nullptr;
SemaphoreHandle_t g_spi1Mutex = nullptr;
SemaphoreHandle_t g_serialMutex = nullptr;
flight_state_t g_flightState = {};
flight_sample_t g_latestSample = {};
system_status_t g_systemStatus = {};
bool g_debugForcePhaseEnabled = false;
uint8_t g_debugForcedPhase = 0;

void serialPrintfSafe(const char *fmt, ...) { va_list a; va_start(a, fmt); vprintf(fmt, a); va_end(a); }
void serialPrintlnSafe(const char *s) { puts(s); }

// sensors.cpp needs real hardware; provide just what flight_logic links against.
namespace sensors
{
  baro_baseline_t baseline() { baro_baseline_t b; b.pressure_pa = 101325.0f; b.temperature_c = 20.0f; return b; }
}

// ---- tiny test framework ----
static int g_failures = 0;
#define CHECK(cond, ...)                                              \
  do {                                                                \
    if (!(cond)) {                                                    \
      ++g_failures;                                                   \
      printf("  FAIL %s:%d: ", __FILE__, __LINE__);                   \
      printf(__VA_ARGS__);                                            \
      printf("\n");                                                   \
    }                                                                 \
  } while (0)

static const char *phaseName(flight_phase_t p)
{
  switch (p) {
  case flight_phase_t::STARTUP: return "STARTUP";
  case flight_phase_t::BOOST: return "BOOST";
  case flight_phase_t::COAST: return "COAST";
  case flight_phase_t::DESCENT: return "DESCENT";
  case flight_phase_t::TOUCHDOWN: return "TOUCHDOWN";
  case flight_phase_t::COMPLETE: return "COMPLETE";
  }
  return "?";
}

// ---------------------------------------------------------------------------
// Simple 1D trajectory simulator. Body +Z is up, so the accelerometer reads
// specific force f_z = a_z + g. Noise is deterministic (LCG) so runs repeat.
// ---------------------------------------------------------------------------
struct sim_t
{
  double t = 0.0, alt = 0.0, vel = 0.0;
  bool chute_out = false;
  uint32_t rng = 12345;
  double noise(double sigma)
  {
    rng = rng * 1664525u + 1013904223u;
    const double u = ((rng >> 8) & 0xFFFF) / 65535.0 - 0.5;
    return u * 2.0 * sigma * 1.7; // ~uniform with similar sigma
  }
  // Returns body-frame vertical acceleration (excluding gravity) for this instant.
  double accel_cmd() const
  {
    if (t < 3.0) return 0.0;                 // pad
    if (t < 5.0) return 60.0;                // boost: 2 s at ~6 g
    if (chute_out) {
      // under canopy: approach -5 m/s terminal velocity
      return (-5.0 - vel) * 2.0;
    }
    return -9.80665;                         // ballistic coast/fall (no drag)
  }
  void step(double dt)
  {
    double a = accel_cmd();
    if (alt <= 0.0 && t > 5.0 && vel < 0.0) { // on the ground
      alt = 0.0; vel = 0.0; a = 0.0;
    }
    vel += a * dt;
    alt += vel * dt;
    if (alt < 0.0) { alt = 0.0; vel = 0.0; }
    t += dt;
  }
  flight_sample_t sample(uint32_t ms, uint32_t idx)
  {
    flight_sample_t s;
    s.timestamp_ms = ms;
    const double a = (alt <= 0.0 && t > 5.0) ? 0.0 : accel_cmd();
    s.accel_mss[0] = static_cast<float>(noise(0.05));
    s.accel_mss[1] = static_cast<float>(noise(0.05));
    s.accel_mss[2] = static_cast<float>(a + 9.80665 + noise(0.05));
    s.gyro_rads[0] = static_cast<float>(noise(0.005));
    s.gyro_rads[1] = static_cast<float>(noise(0.005));
    s.gyro_rads[2] = static_cast<float>(noise(0.005));
    s.baro_fresh = (idx % app_config::kBaroTemperatureEveryN) != 0U;
    s.altitude_m = static_cast<float>(alt + noise(0.3));
    s.pressure_pa = static_cast<float>(101325.0 * pow(1.0 - 2.25577e-5 * alt, 5.25588));
    s.temperature_c = 20.0f;
    return s;
  }
};

struct flight_result_t
{
  uint32_t boost_ms = 0, coast_ms = 0, descent_ms = 0, touchdown_ms = 0, complete_ms = 0;
  uint32_t pyro_on_ms = 0, pyro_off_ms = 0;
  double alt_at_fire = -1.0;
  double max_alt = 0.0;
  deploy_reason_t reason = deploy_reason_t::NONE;
};

static void run_sim(sim_t &sim, flight_result_t &r, double until_s,
                    bool run_supervisor,
                    double reset_at_s = -1.0, bool *did_reset = nullptr)
{
  const double dt = app_config::kSensorPeriodMs / 1000.0;
  uint32_t idx = 0;
  flight_phase_t last_phase = g_flightState.phase;
  bool pyro_was_on = false;
  while (sim.t < until_s)
  {
    g_host_millis += app_config::kSensorPeriodMs;
    sim.step(dt);
    flight_sample_t s = sim.sample(g_host_millis, idx++);
    flight_logic::update(s, g_flightState);
    supervisor::heartbeatSensor();
    supervisor::heartbeatFlight(100);

    if (run_supervisor && (idx % (app_config::kSupervisorPeriodMs / app_config::kSensorPeriodMs)) == 0)
    {
      // run one supervisor iteration inline (the task body loops forever)
      extern void supervisor_test_step();
      supervisor_test_step();
    }

    if (g_flightState.parachute_deployed && !sim.chute_out) sim.chute_out = true;
    if (getenv("SIM_TRACE") && (idx % 100) == 0)
      printf("    trace t=%6.2f true alt=%7.1f vel=%7.1f | est alt=%7.1f vel=%7.1f bias=%6.2f baroOK=%d nis=%6.1f phase=%s\n",
             sim.t, sim.alt, sim.vel, g_flightState.deployment_altitude_m, g_flightState.deployment_velocity_mps,
             g_flightState.vertical_filter.accel_bias_mss, g_flightState.vertical_filter.baro_update_accepted,
             g_flightState.vertical_filter.normalized_baro_innovation, phaseName(g_flightState.phase));
    if (sim.alt > r.max_alt) r.max_alt = sim.alt;

    if (g_flightState.phase != last_phase)
    {
      printf("  t=%6.2fs alt=%7.1f vel=%7.1f  %s -> %s\n", sim.t, sim.alt, sim.vel,
             phaseName(last_phase), phaseName(g_flightState.phase));
      last_phase = g_flightState.phase;
      switch (last_phase) {
      case flight_phase_t::BOOST: r.boost_ms = g_host_millis; break;
      case flight_phase_t::COAST: r.coast_ms = g_host_millis; break;
      case flight_phase_t::DESCENT: r.descent_ms = g_host_millis; break;
      case flight_phase_t::TOUCHDOWN: r.touchdown_ms = g_host_millis; break;
      case flight_phase_t::COMPLETE: r.complete_ms = g_host_millis; break;
      default: break;
      }
    }
    const bool pyro_on = g_host_pin_state[app_config::kPyro1Pin] == arduino::HIGH;
    if (pyro_on && !pyro_was_on) {
      r.pyro_on_ms = g_host_millis; r.alt_at_fire = sim.alt; r.reason = g_flightState.deploy_reason;
      printf("  t=%6.2fs alt=%7.1f vel=%7.1f  PYRO ON (reason %u)\n", sim.t, sim.alt, sim.vel,
             static_cast<unsigned>(r.reason));
    }
    if (!pyro_on && pyro_was_on) { r.pyro_off_ms = g_host_millis; printf("  t=%6.2fs PYRO OFF\n", sim.t); }
    pyro_was_on = pyro_on;

    if (reset_at_s > 0.0 && sim.t >= reset_at_s && did_reset && !*did_reset)
    {
      *did_reset = true;
      return; // caller simulates the reboot
    }
  }
}

// The supervisor task body loops forever; expose one iteration for tests.
namespace supervisor { void testStep(); }
void supervisor_test_step() { supervisor::testStep(); }

// ---------------------------------------------------------------------------
static void test_packets()
{
  printf("[packets]\n");
  telemetry::inputs_t in;
  in.state.phase = flight_phase_t::COAST;
  in.state.pyro_armed = true;
  in.state.altitude_m = 1234.56;
  in.state.velocity_mps = -12.345;
  in.state.orientation_q = {0.70710678, 0.0, 0.70710678, 0.0};
  in.state.gps.valid = true;
  in.state.gps.latitude_deg = 39.9522;
  in.state.gps.longitude_deg = -75.1932;
  in.state.gps.altitude_valid = true;
  in.state.gps.altitude_m = 1300.25;
  in.state.gps.fix_type = 3;
  in.state.gps.satellites = 11;
  in.state.gps.p_dop = 1.75;
  in.state.gps.horizontal_accuracy_m = 2.5;
  in.state.deploy_reason = deploy_reason_t::NONE;
  in.state.estimator.imu_calibration_complete = true;
  in.state.estimator.accel_bias_mss[0] = 0.012; in.state.estimator.accel_bias_mss[1] = -0.034; in.state.estimator.accel_bias_mss[2] = 0.056;
  in.state.estimator.gyro_bias_rads[0] = 0.0012; in.state.estimator.gyro_bias_rads[1] = -0.0034; in.state.estimator.gyro_bias_rads[2] = 0.0056;
  in.state.estimator.normalized_baro_innovation = 1.23;
  in.state.estimator.ahrs_initialized = true;
  in.state.vertical_filter.baro_update_accepted = true;
  in.state.vertical_filter.accel_bias_mss = 0.078;
  in.state.vertical_filter.normalized_baro_innovation = -0.5;
  in.state.position_m = {10.5, -20.25, 1234.0};
  in.sample.altitude_m = 1230.0f;
  in.sample.accel_mss[0] = 0.1f; in.sample.accel_mss[1] = -0.2f; in.sample.accel_mss[2] = 9.81f;
  in.sample.gyro_rads[0] = 0.01f; in.sample.gyro_rads[1] = -0.02f; in.sample.gyro_rads[2] = 0.03f;
  in.sample.mag_raw[0] = 100; in.sample.mag_raw[1] = -200; in.sample.mag_raw[2] = 300;
  in.sample.pressure_pa = 87654.3f;
  in.sample.temperature_c = 21.5f;
  in.system.sd_logging_enabled = true;
  in.system.radio_initialized = true;
  in.system.reset_cause = 0x11;
  in.system.restore_count = 0;
  in.supervisor.watchdog_armed = true;
  in.supervisor.heartbeats_ok = true;
  in.supervisor.backup_timer_armed = true;
  in.supervisor.backup_timer_remaining_ms = 4321;
  in.supervisor.sensor_rate_hz = 100;
  in.supervisor.flight_rate_hz = 100;
  in.supervisor.flight_update_max_us = 850;
  in.radio_tx_count = 777;
  in.radio_tx_fail = 2;

  tlm::state_packet_t sp;
  tlm::health_packet_t hp;
  telemetry::buildStatePacket(123456, in, 7, sp);
  telemetry::buildHealthPacket(123456, in, 9, hp);
  CHECK(tlm::verify(sp), "state crc");
  CHECK(tlm::verify(hp), "health crc");
  CHECK(sp.alt_cm == 123456, "alt_cm=%d", sp.alt_cm);
  CHECK(sp.vel_cms == -1235, "vel_cms=%d", sp.vel_cms);
  CHECK(sp.q[0] == 7071, "q0=%d", sp.q[0]);
  CHECK(sp.lat_1e7 == 399522000, "lat=%d", sp.lat_1e7);
  CHECK((sp.flags & tlm::kFlagPyroArmed) && (sp.flags & tlm::kFlagGpsValid) && (sp.flags & tlm::kFlagBackupArmed), "flags=%02x", sp.flags);
  CHECK(hp.gps_pdop_x100 == 175, "pdop=%u", hp.gps_pdop_x100);
  CHECK(hp.sensor_hz == 100, "sensor_hz");

  FILE *f = fopen("state.bin", "wb"); fwrite(&sp, 1, sizeof(sp), f); fclose(f);
  f = fopen("health.bin", "wb"); fwrite(&hp, 1, sizeof(hp), f); fclose(f);
  // Expected values for the Python cross-check.
  f = fopen("expected.json", "w");
  fprintf(f, "{\"state\":{\"type\":1,\"seq\":7,\"t_ms\":123456,\"phase\":2,\"alt_cm\":%d,\"vel_cms\":%d,"
             "\"baro_alt_cm\":%d,\"q\":[%d,%d,%d,%d],\"lat_1e7\":%d,\"lon_1e7\":%d,\"accel_mag_cmss\":%d,"
             "\"flags\":%u,\"deploy_reason\":0},"
             "\"health\":{\"type\":2,\"seq\":9,\"t_ms\":123456,\"pressure_pa\":%d,\"temp_cc\":%d,"
             "\"gps_sats\":%u,\"gps_pdop_x100\":%u,\"status\":%u,\"sensor_hz\":%u,\"flight_update_max_us\":%u,"
             "\"backup_remaining_ms\":%u,\"radio_tx_count\":%u}}\n",
          sp.alt_cm, sp.vel_cms, sp.baro_alt_cm, sp.q[0], sp.q[1], sp.q[2], sp.q[3], sp.lat_1e7, sp.lon_1e7,
          sp.accel_mag_cmss, sp.flags, hp.pressure_pa, hp.temp_cc, hp.gps_sats, hp.gps_pdop_x100, hp.status,
          hp.sensor_hz, hp.flight_update_max_us, hp.backup_remaining_ms, hp.radio_tx_count);
  fclose(f);
  printf("  state %zu bytes, health %zu bytes written\n", sizeof(sp), sizeof(hp));
}

static void fresh_boot()
{
  g_flightState = flight_state_t{};
  g_systemStatus = system_status_t{};
  memset(g_host_pin_state, 0, sizeof(g_host_pin_state));
  supervisor::init();
  flight_logic::init();
}

static void test_nominal_flight()
{
  printf("[nominal flight]\n");
  g_host_millis = 1000;
  fresh_boot();
  persist::clear();
  sim_t sim;
  flight_result_t r;
  run_sim(sim, r, 90.0, true);

  CHECK(r.boost_ms > 0 && r.coast_ms > r.boost_ms && r.descent_ms > r.coast_ms, "phase order");
  CHECK(r.touchdown_ms > r.descent_ms && r.complete_ms > r.touchdown_ms, "landed and completed");
  CHECK(r.pyro_on_ms > 0, "pyro fired");
  CHECK(r.alt_at_fire <= app_config::kDeployAltM + 5.0 && r.alt_at_fire > 50.0, "fired at %.1f m", r.alt_at_fire);
  CHECK(r.reason == deploy_reason_t::ALTITUDE, "reason %u", (unsigned)r.reason);
  CHECK(r.pyro_off_ms - r.pyro_on_ms >= app_config::kPyroOnMs && r.pyro_off_ms - r.pyro_on_ms <= app_config::kPyroOnMs + 60,
        "pyro on for %u ms", r.pyro_off_ms - r.pyro_on_ms);
  // Confirmation windows: boost detected within ~kLaunchConfirmMs of t=3 s
  CHECK(r.boost_ms >= 1000 + 3000 + app_config::kLaunchConfirmMs - 20 && r.boost_ms <= 1000 + 3000 + app_config::kLaunchConfirmMs + 60,
        "boost at +%u ms", r.boost_ms - 1000);
  CHECK(sim.chute_out, "sim saw chute");
  // Record must be cleared after COMPLETE
  persist::record_t rec;
  CHECK(!persist::load(rec), "record cleared after COMPLETE");
  printf("  max alt %.0f m\n", r.max_alt);
}

static void test_watchdog_restore()
{
  printf("[watchdog reset during coast]\n");
  g_host_millis = 1000;
  fresh_boot();
  persist::clear();
  sim_t sim;
  flight_result_t r;
  bool did_reset = false;
  run_sim(sim, r, 90.0, true, 12.0, &did_reset);
  CHECK(did_reset && g_flightState.phase == flight_phase_t::COAST, "reset happened in COAST");

  // --- simulated reboot ---
  persist::record_t rec;
  CHECK(persist::load(rec), "record present");
  CHECK(persist::recordIsInFlight(rec), "record is in flight");
  const flight_state_t before = g_flightState;
  g_host_millis += 400; // reboot time
  g_host_SRC_SRSR = (1u << 4);
  fresh_boot();
  const uint8_t cause = persist::readAndClearResetCause();
  CHECK(persist::resetCauseWasWatchdog(cause), "cause is watchdog");
  g_systemStatus.restore_count = rec.restore_count + 1;
  flight_logic::restore(rec, g_flightState, g_host_millis);
  CHECK(g_flightState.phase == flight_phase_t::COAST, "restored phase");
  CHECK(fabs(g_flightState.home_altitude_m - before.home_altitude_m) < 1e-3, "home altitude restored");
  CHECK(g_flightState.restored_from_persist, "restored flag");
  CHECK(g_flightState.launch_elapsed_before_reset_ms > 8000, "elapsed restored (%u)", g_flightState.launch_elapsed_before_reset_ms);

  flight_result_t r2;
  run_sim(sim, r2, 90.0, true);
  CHECK(r2.pyro_on_ms > 0, "pyro fired after restore");
  CHECK(r2.alt_at_fire <= app_config::kDeployAltM + 10.0 && r2.alt_at_fire > 30.0, "fired at %.1f m after restore", r2.alt_at_fire);
  CHECK(r2.touchdown_ms > 0, "touchdown after restore");

  // A stale record after a *power-on* reset must NOT be in-flight-restorable
  g_host_SRC_SRSR = (1u << 0);
  const uint8_t por = persist::readAndClearResetCause();
  CHECK(!persist::resetCauseWasWatchdog(por), "POR is not watchdog");
}

static void test_backup_timer_and_shutoff()
{
  printf("[backup timer + supervisor shut-off]\n");
  // Backup timer is 0 (disabled) in app_config by design; exercise forceDeploy and pyroOff directly.
  g_host_millis = 5000;
  fresh_boot();
  g_flightState.phase = flight_phase_t::COAST;
  g_flightState.launch_time_ms = 1000;
  flight_logic::forceDeploy(g_flightState, g_host_millis, deploy_reason_t::BACKUP_TIMER);
  CHECK(g_flightState.parachute_deployed && g_flightState.pyro_active, "force deploy");
  CHECK(g_flightState.phase == flight_phase_t::DESCENT, "forced into DESCENT");
  CHECK(g_host_pin_state[app_config::kPyro1Pin] == arduino::HIGH, "pin high");
  // second call is a no-op
  flight_logic::forceDeploy(g_flightState, g_host_millis, deploy_reason_t::BACKUP_TIMER);
  CHECK(g_flightState.deploy_reason == deploy_reason_t::BACKUP_TIMER, "reason kept");
  // supervisor shut-off with no new samples
  g_host_millis += app_config::kPyroOnMs + 10;
  supervisor::testStep();
  CHECK(!g_flightState.pyro_active, "supervisor turned pyro off");
  CHECK(g_host_pin_state[app_config::kPyro1Pin] == arduino::LOW, "pin low");

  // Heartbeat starvation must stop watchdog feeds
  supervisor::init();
  g_host_millis += app_config::kHeartbeatGraceAfterBootMs + 10;
  supervisor::heartbeatSensor(); supervisor::heartbeatFlight(10);
  supervisor::testStep();
  const uint32_t feeds = supervisor::status().watchdog_feeds;
  g_host_millis += app_config::kHeartbeatFlightMaxMs + 100; // flight task "dies"
  supervisor::heartbeatSensor();
  supervisor::testStep();
  CHECK(supervisor::status().watchdog_feeds == feeds, "no feed while flight task stale");
  CHECK(!supervisor::status().heartbeats_ok, "heartbeats flagged");
}

int main()
{
  test_packets();
  test_nominal_flight();
  test_watchdog_restore();
  test_backup_timer_and_shutoff();
  if (g_failures == 0) { printf("ALL HOST TESTS PASSED\n"); return 0; }
  printf("%d FAILURE(S)\n", g_failures);
  return 1;
}
