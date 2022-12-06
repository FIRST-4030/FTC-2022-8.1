package org.firstinspires.ftc.teamcode.robot.powerplay2022.teleop.production;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc.InputAutoMapper;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager.Conditional;
import org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager.TaskManager;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@TeleOp(name = "TaskManagerStateMachineDemo", group = "testers")
public class TaskManagerStateMachineDemo extends LoopUtil {

    private enum ANIMAL{
        BUNNY,
        DOLPHIN,
        AMOEBA,
        BEAR
    }

    public TaskManager taskManager;

    public static Telemetry console;

    public static InputHandler inputHandler;

    public static double elapsedTime;

    @Override
    public void opInit() {
        TaskManagerStateMachineDemo.console = telemetry;
        TaskManagerStateMachineDemo.inputHandler = InputAutoMapper.normal.autoMap(this);
        TaskManagerStateMachineDemo.elapsedTime = 0;

        taskManager = new TaskManager();

        taskManager.alwaysRun = () -> {inputHandler.loop();};

        taskManager.addStates(
                () -> console.addData("Src: https://www.asciiart.eu/animals/rabbits", "\nArt by Joan Stark\n" +
                        "             ,\\\n" +
                        "             \\\\\\,_\n" +
                        "              \\` ,\\\n" +
                        "         __,.-\" =__)\n" +
                        "       .\"        )\n" +
                        "    ,_/   ,    \\/\\_\n" +
                        "    \\_|    )_-\\ \\_-`\n" +
                        "jgs    `-----` `--`"),
                () -> console.addData("Src: https://www.asciiart.eu/animals/dolphins", "\nArt by Morfina\n" +
                        "              ,-.\n" +
                        "             /  (  '\n" +
                        "     *  _.--'!   '--._\n" +
                        "      ,'              ''.\n" +
                        "'    |!                   \\\n" +
                        "   _.'  O      ___       ! \\\n" +
                        "  (_.-^, __..-'  ''''--.   )\n" +
                        "      /,'        '    _.' /\n" +
                        "   '         *    .-''    |\n" +
                        "                 (..--^.  ' \n" +
                        "                  mrf  | /\n" +
                        "                       '"),
                () -> console.addData("Src: https://www.asciiart.eu/animals/amoeba", "\n             ,,,,,,,,\n" +
                        "           ,|||````||||\n" +
                        "     ,,,,|||||       ||,\n" +
                        "  ,||||```````       `||\n" +
                        ",|||`                 |||,\n" +
                        "||`     ....,          `|||\n" +
                        "||     ::::::::          |||,\n" +
                        "||     :::::::'     ||    ``|||,\n" +
                        "||,     :::::'               `|||\n" +
                        "`||,                           |||\n" +
                        " `|||,       ||          ||    ,||\n" +
                        "   `||                        |||`\n" +
                        "    ||                   ,,,||||\n" +
                        "    ||              ,||||||```\n" +
                        "   ,||         ,,|||||`\n" +
                        "  ,||`   ||   |||`\n" +
                        " |||`         ||\n" +
                        ",||           ||\n" +
                        "||`           ||\n" +
                        "|||,         |||\n" +
                        " `|||,,    ,|||\n" +
                        "   ``||||||||`"),
                () -> console.addData("Src: https://www.asciiart.eu/animals/bears",
                        "\n __         __\n" +
                        "/  \\.-\"\"\"-./  \\\n" +
                        "\\    -   -    /\n" +
                        " |   o   o   |\n" +
                        " \\  .-'''-.  /\n" +
                        "  '-\\__Y__/-'\n" +
                        "     `---`")
        );

        taskManager.addConditions(
                   new Conditional() {
                    @Override
                    public void init() {
                        linkedStates = new int[]{ANIMAL.BEAR.ordinal(), ANIMAL.DOLPHIN.ordinal()};
                    }

                    @Override
                    public void check() {
                        if (TaskManagerStateMachineDemo.inputHandler.held("D1:RT") && TaskManagerStateMachineDemo.elapsedTime >= 1000) {
                            status = STATUS.PASSED;
                        } else {
                            status = STATUS.FAILED;
                        }
                    }
                }, new Conditional() {
                    @Override
                    public void init() {
                        linkedStates = new int[]{ANIMAL.AMOEBA.ordinal()};
                    }

                    @Override
                    public void check() {
                        if (TaskManagerStateMachineDemo.inputHandler.held("D1:LT") && TaskManagerStateMachineDemo.elapsedTime >= 2000) {
                            status = STATUS.PASSED;
                        } else {
                            status = STATUS.FAILED;
                        }
                    }
                }, new Conditional() {
                    @Override
                    public void init() {
                        linkedStates = new int[]{ANIMAL.BUNNY.ordinal(), ANIMAL.DOLPHIN.ordinal()};
                    }

                    @Override
                    public void check() {
                        if (gamepad1.a && TaskManagerStateMachineDemo.elapsedTime >= 2500) {
                            status = STATUS.PASSED;
                        } else {
                            status = STATUS.FAILED;
                        }
                    }
                });
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        elapsedTime += deltaTime;
        taskManager.execute();
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
