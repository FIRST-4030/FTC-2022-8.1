package org.firstinspires.ftc.teamcode.utils.gamepad;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021.Globals;

@Disabled
@TeleOp(name = "InputHandlerTest", group = "Test")
public class InputHandlerTest extends OpMode {
    // Class under test
    private InputHandler in;

    // Timestamps and last values
    private long active;
    private boolean lastActive;
    private long down;
    private boolean lastDown;
    private long up;
    private boolean lastUp;
    private long changed;
    private boolean lastChanged;
    private long auto;
    private boolean lastAuto;
    private long toggle;
    private boolean lastToggle;
    private long held;
    private boolean lastHeld;
    private long analog;
    private float lastAnalog;
    private long active2;
    private boolean lastActive2;
    private long down2;
    private boolean lastDown2;
    private long up2;
    private boolean lastUp2;
    private long changed2;
    private boolean lastChanged2;
    private long auto2;
    private boolean lastAuto2;
    private long toggle2;
    private boolean lastToggle2;
    private long held2;
    private boolean lastHeld2;
    private long analog2;
    private float lastAnalog2;

    @Override
    public void init() {
        Globals.opmode(this); // Just in case someone calls Globals

        in = new InputHandler(this);
        in.register("BINARY", GAMEPAD.driver1, PAD_KEY.a);
        in.register("BINARY2", GAMEPAD.driver2, PAD_KEY.a);
        in.register("ANALOG", GAMEPAD.driver1, PAD_KEY.left_stick_x);
        in.register("ANALOG2", GAMEPAD.driver2, PAD_KEY.left_stick_x);

        telemetry.clearAll();
    }

    @Override
    public void init_loop() {
        in.loop();
        telemetry.addData(getClass().getSimpleName(), "Ready");
        telemetry.addData("A", gamepad1.a);
        telemetry.addData("LX", gamepad1.left_stick_x);
        telemetry.addData("A2", gamepad2.a);
        telemetry.addData("LX2", gamepad2.left_stick_x);
        telemetry.addData("BINARY", in.active("BINARY"));
        telemetry.addData("ANALOG", in.value("ANALOG"));
        telemetry.addData("BINARY2", in.active("BINARY2"));
        telemetry.addData("ANALOG2", in.value("ANALOG2"));
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.clearAll();
    }

    @Override
    public void loop() {
        // Copy the current time in so we don't make repeated system calls
        long now = System.currentTimeMillis();

        // Repeated calls to in.down() and friends are guaranteed to return the same value
        // This means you don't have to copy the value into your local context
        // Calling InputHandler.loop() may change the values, and should be done first
        in.loop();

        // Collect timestamps for changes
        if (in.active("BINARY") != lastActive) {
            active = now;
        }
        lastActive = in.active("BINARY");
        if (in.down("BINARY") != lastDown) {
            down = now;
        }
        lastDown = in.down("BINARY");
        if (in.up("BINARY") != lastUp) {
            up = now;
        }
        lastUp = in.up("BINARY");
        if (in.changed("BINARY") != lastChanged) {
            changed = now;
        }
        lastChanged = in.changed("BINARY");
        if (in.auto("BINARY") != lastAuto) {
            auto = now;
        }
        lastAuto = in.auto("BINARY");
        if (in.toggle("BINARY") != lastToggle) {
            toggle = now;
        }
        lastToggle = in.toggle("BINARY");
        if (in.held("BINARY") != lastHeld) {
            held = now;
        }
        lastHeld = in.held("BINARY");
        if (in.value("ANALOG") != lastAnalog) {
            analog = now;
        }
        lastAnalog = in.value("ANALOG");

        // Collect timestamps for changes
        if (in.active("BINARY2") != lastActive2) {
            active2 = now;
        }
        lastActive2 = in.active("BINARY2");
        if (in.down("BINARY2") != lastDown2) {
            down2 = now;
        }
        lastDown2 = in.down("BINARY2");
        if (in.up("BINARY2") != lastUp2) {
            up2 = now;
        }
        lastUp2 = in.up("BINARY2");
        if (in.changed("BINARY2") != lastChanged2) {
            changed2 = now;
        }
        lastChanged2 = in.changed("BINARY2");
        if (in.auto("BINARY2") != lastAuto2) {
            auto2 = now;
        }
        lastAuto2 = in.auto("BINARY2");
        if (in.toggle("BINARY2") != lastToggle2) {
            toggle2 = now;
        }
        lastToggle2 = in.toggle("BINARY2");
        if (in.held("BINARY2") != lastHeld2) {
            held2 = now;
        }
        lastHeld2 = in.held("BINARY2");
        if (in.value("ANALOG2") != lastAnalog2) {
            analog2 = now;
        }
        lastAnalog2 = in.value("ANALOG2");

        // Print the current value and the time of last change [relative to down(D) and now(N)]
        telemetry.addData("active", "%d\tD %d\tN %d",
                in.active("BINARY") ? 1 : 0, active - down, now - active);
        telemetry.addData("down", "%d\tD %d\tN %d",
                in.down("BINARY") ? 1 : 0, 0, now - down);
        telemetry.addData("up", "%d\tD %d\tN %d",
                in.up("BINARY") ? 1 : 0, up - down, now - up);
        telemetry.addData("changed", "%d\tD %d\tN %d",
                in.changed("BINARY") ? 1 : 0, changed - down, now - changed);
        telemetry.addData("auto", "%d\tD %d\tN %d",
                in.auto("BINARY") ? 1 : 0, auto - down, now - auto);
        telemetry.addData("toggle", "%d\tD %d\tN %d",
                in.toggle("BINARY") ? 1 : 0, toggle - down, now - toggle);
        telemetry.addData("held", "%d\tD %d\tN %d",
                in.held("BINARY") ? 1 : 0, held - down, now - held);
        telemetry.addData("analog", "%.2f\tD %d\tN %d",
                in.value("ANALOG"), analog - down, now - analog);

        telemetry.addData("active2", "%d\tD %d\tN %d",
                in.active("BINARY2") ? 1 : 0, active2 - down2, now - active2);
        telemetry.addData("down2", "%d\tD %d\tN %d",
                in.down("BINARY2") ? 1 : 0, 0, now - down2);
        telemetry.addData("up2", "%d\tD %d\tN %d",
                in.up("BINARY2") ? 1 : 0, up2 - down2, now - up2);
        telemetry.addData("changed2", "%d\tD %d\tN %d",
                in.changed("BINARY2") ? 1 : 0, changed2 - down2, now - changed2);
        telemetry.addData("auto2", "%d\tD %d\tN %d",
                in.auto("BINARY2") ? 1 : 0, auto2 - down2, now - auto2);
        telemetry.addData("toggle2", "%d\tD %d\tN %d",
                in.toggle("BINARY2") ? 1 : 0, toggle2 - down2, now - toggle2);
        telemetry.addData("held2", "%d\tD %d\tN %d",
                in.held("BINARY2") ? 1 : 0, held2 - down2, now - held2);
        telemetry.addData("analog2", "%.2f\tD %d\tN %d",
                in.value("ANALOG2"), analog2 - down2, now - analog2);
        telemetry.update();
    }
}
