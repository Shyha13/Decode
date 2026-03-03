package org.firstinspires.ftc.teamcode.opmodes.prod;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utils.Prism.Color;

@Autonomous(name = "Close Side Auto Path", group = "Prod")
@Config
public class CloseAutoPath extends ClosePath {
    public CloseAutoPath() {
        super("BLUE");
    }
}
