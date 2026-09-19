import test from "node:test";
import assert from "node:assert/strict";
import {formatMotionSpeedDiagnostics as format} from "../motion_diagnostics.js";

test("missing, idle, stale and unavailable telemetry cannot retain a false speed percentage", () => {
  assert.match(format(null).title, /等待/);
  assert.match(format({measurement_valid:true}).title, /无有效/);
  assert.match(format({command_stale:true}).title, /过期/);
  assert.match(format({measurement_valid:false}).title, /不可用/);
});
test("reports directional speed, independent units and evidence-backed joint reason", () => {
  const result=format({measurement_valid:true,
    linear:{active:true,warning:true,expected_speed:.05,actual_speed_along_command:.03,ratio:.6,
      reasons:[{code:"joint_position_limit",joints:[4]}]},
    angular:{active:true,warning:false,expected_speed:.1,actual_speed_along_command:.1,ratio:1,reasons:[]}});
  assert.equal(result.warning,true);
  assert.match(result.detail,/60%/); assert.match(result.detail,/关节 4/);
  assert.match(result.detail,/m\/s/); assert.match(result.detail,/rad\/s/);
});
test("unknown causes and solver status are rendered as plain text", () => {
  const result=format({measurement_valid:true,linear:{active:true,warning:true,ratio:0,expected_speed:.05,
    actual_speed_along_command:0,reasons:[{code:"future_cause",detail:"<script>"}]}});
  assert.match(result.detail,/原因待确认/);
  assert.match(result.detail,/<script>/); // app renders with textContent, never innerHTML
});
