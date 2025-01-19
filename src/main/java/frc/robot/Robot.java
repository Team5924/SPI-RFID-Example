// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in the TimedRobot documentation. If you change the name of
 * this class or the package after creating this project, you must also update
 * the Main.java file in the project.
 */
public class Robot extends TimedRobot {

    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    /* ##### SPI Stuff ##### */
    // Constants
    private static final byte PN532_PREAMBLE = (byte) 0x00;   ///< Command sequence start, byte 1/3
    private static final byte PN532_STARTCODE1 = (byte) 0x00; ///< Command sequence start, byte 2/3
    private static final byte PN532_STARTCODE2 = (byte) 0xFF; ///< Command sequence start, byte 3/3
    private static final byte PN532_POSTAMBLE = (byte) 0x00;  ///< EOD
    
    private static final byte PN532_HOSTTOPN532 = (byte) 0xD4; ///< Host-to-PN532
    private static final byte PN532_PN532TOHOST = (byte) 0xD5; ///< PN532-to-host
    
    // PN532 Commands
    private static final byte PN532_COMMAND_DIAGNOSE = (byte) 0x00;              ///< Diagnose
    private static final byte PN532_COMMAND_GETFIRMWAREVERSION = (byte) 0x02;    ///< Get firmware version
    private static final byte PN532_COMMAND_GETGENERALSTATUS = (byte) 0x04;      ///< Get general status
    private static final byte PN532_COMMAND_READREGISTER = (byte) 0x06;         ///< Read register
    private static final byte PN532_COMMAND_WRITEREGISTER = (byte) 0x08;         ///< Write register
    private static final byte PN532_COMMAND_READGPIO = (byte) 0x0C;              ///< Read GPIO
    private static final byte PN532_COMMAND_WRITEGPIO = (byte) 0x0E;             ///< Write GPIO
    private static final byte PN532_COMMAND_SETSERIALBAUDRATE = (byte) 0x10;     ///< Set serial baud rate
    private static final byte PN532_COMMAND_SETPARAMETERS = (byte) 0x12;         ///< Set parameters
    private static final byte PN532_COMMAND_SAMCONFIGURATION = (byte) 0x14;      ///< SAM configuration
    private static final byte PN532_COMMAND_POWERDOWN = (byte) 0x16;             ///< Power down
    private static final byte PN532_COMMAND_RFCONFIGURATION = (byte) 0x32;       ///< RF config
    private static final byte PN532_COMMAND_RFREGULATIONTEST = (byte) 0x58;      ///< RF regulation test
    private static final byte PN532_COMMAND_INJUMPFORDEP = (byte) 0x56;          ///< Jump for DEP
    private static final byte PN532_COMMAND_INJUMPFORPSL = (byte) 0x46;          ///< Jump for PSL
    private static final byte PN532_COMMAND_INLISTPASSIVETARGET = (byte) 0x4A;   ///< List passive target
    private static final byte PN532_COMMAND_INATR = (byte) 0x50;                 ///< ATR
    private static final byte PN532_COMMAND_INPSL = (byte) 0x4E;                 ///< PSL
    private static final byte PN532_COMMAND_INDATAEXCHANGE = (byte) 0x40;        ///< Data exchange
    private static final byte PN532_COMMAND_INCOMMUNICATETHRU = (byte) 0x42;     ///< Communicate through
    private static final byte PN532_COMMAND_INDESELECT = (byte) 0x44;            ///< Deselect
    private static final byte PN532_COMMAND_INRELEASE = (byte) 0x52;             ///< Release
    private static final byte PN532_COMMAND_INSELECT = (byte) 0x54;              ///< Select
    private static final byte PN532_COMMAND_INAUTOPOLL = (byte) 0x60;            ///< Auto poll
    private static final byte PN532_COMMAND_TGINITASTARGET = (byte) 0x8C;        ///< Init as target
    private static final byte PN532_COMMAND_TGSETGENERALBYTES = (byte) 0x92;     ///< Set general bytes
    private static final byte PN532_COMMAND_TGGETDATA = (byte) 0x86;             ///< Get data
    private static final byte PN532_COMMAND_TGSETDATA = (byte) 0x8E;             ///< Set data
    private static final byte PN532_COMMAND_TGSETMETADATA = (byte) 0x94;         ///< Set metadata
    private static final byte PN532_COMMAND_TGGETINITIATORCOMMAND = (byte) 0x88; ///< Get initiator command
    private static final byte PN532_COMMAND_TGRESPONSETOINITIATOR = (byte) 0x90; ///< Response to initiator
    private static final byte PN532_COMMAND_TGGETTARGETSTATUS = (byte) 0x8A;     ///< Get target status
    
    private static final byte PN532_RESPONSE_INDATAEXCHANGE = (byte) 0x41;      ///< Data exchange
    private static final byte PN532_RESPONSE_INLISTPASSIVETARGET = (byte) 0x4B; ///< List passive target
    
    private static final byte PN532_WAKEUP = (byte) 0x55; ///< Wake
    
    private static final byte PN532_SPI_STATREAD = (byte) 0x02;  ///< Stat read
    private static final byte PN532_SPI_DATAWRITE = (byte) 0x01; ///< Data write
    private static final byte PN532_SPI_DATAREAD = (byte) 0x03;  ///< Data read
    private static final byte PN532_SPI_READY = (byte) 0x01;     ///< Ready
    
    private static final byte PN532_MIFARE_ISO14443A = (byte) 0x00; ///< MiFare
    
    // Mifare Commands
    private static final byte MIFARE_CMD_AUTH_A = (byte) 0x60;           ///< Auth A
    private static final byte MIFARE_CMD_AUTH_B = (byte) 0x61;           ///< Auth B
    private static final byte MIFARE_CMD_READ = (byte) 0x30;             ///< Read
    private static final byte MIFARE_CMD_WRITE = (byte) 0xA0;            ///< Write
    private static final byte MIFARE_CMD_TRANSFER = (byte) 0xB0;         ///< Transfer
    private static final byte MIFARE_CMD_DECREMENT = (byte) 0xC0;        ///< Decrement
    private static final byte MIFARE_CMD_INCREMENT = (byte) 0xC1;        ///< Increment
    private static final byte MIFARE_CMD_STORE = (byte) 0xC2;            ///< Store
    private static final byte MIFARE_ULTRALIGHT_CMD_WRITE = (byte) 0xA2l; ///< Write (MiFare Ultralight)

    // SPI Objects
    private SPI spi;

    // Threads and stuff
    // Functions

    void writecommand(Byte[] cmd, int cmdlen) {
        // SPI command write.
        int checksum;
        Byte[] p = new Byte[9 + cmdlen];
        cmdlen++;

        p[0] = PN532_SPI_DATAWRITE;
        p++;

        p[0] = PN532_PREAMBLE;
        p++;
        p[0] = PN532_STARTCODE1;
        p++;
        p[0] = PN532_STARTCODE2;
        p++;
        checksum = PN532_PREAMBLE + PN532_STARTCODE1 + PN532_STARTCODE2;

        p[0] = cmdlen;
        p++;
        p[0] = ~cmdlen + 1;
        p++;

        p[0] = PN532_HOSTTOPN532;
        p++;
        checksum += PN532_HOSTTOPN532;

        for (uint8_t i = 0; i < cmdlen - 1; i++) {
            p[0] = cmd[i];
            p++;
            checksum += cmd[i];
        }

        p[0] = ~checksum;
        p++;
        p[0] = PN532_POSTAMBLE;
        p++;
        spi.write(packet, 8 + cmdlen);
    }

    bool Adafruit_PN532_sendCommandCheckAck(Byte[] cmd, int cmdlen, int timeout) {
        // write the command
        writecommand(cmd, cmdlen);

        // Wait for chip to say its ready!
        if (!waitready(timeout)) {
            return false;
        }
        #ifdef PN532DEBUG
        if (spi_dev == NULL) {
            PN532DEBUGPRINT.println(F("IRQ received"));
        }
        #endif // read acknowledgement
        if (!readack()) {
            #ifdef PN532DEBUG
            PN532DEBUGPRINT.println(F("No ACK frame received!"));
            #endif
            return false;
        }

        // I2C TUNING
        delay(SLOWDOWN);

        // Wait for chip to say its ready!
        if (!waitready(timeout)) {
            return false;
        }

        return true; // ack'd command
    }
}
/**
 * This function is run when the robot is first started up and should be used
 * for any initialization code.
 */
public Robot() {
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);

        spi = new SPI(Port.kOnboardCS0);
        spi.setClockRate(1000000);
        spi.setMode(SPI.Mode.kMode3);
        spi.setChipSelectActiveLow();

    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
public void robotPeriodic() {
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
public void autonomousInit() {
        m_autoSelected = m_chooser.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + m_autoSelected);
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
public void autonomousPeriodic() {
        switch (m_autoSelected) {
            case kCustomAuto:
                // Put custom auto code here
                break;
            case kDefaultAuto:
            default:
                // Put default auto code here
                break;
        }
    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
public void teleopInit() {
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
public void teleopPeriodic() {
    }

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
public void disabledInit() {
    }

    /**
     * This function is called periodically when disabled.
     */
    @Override
public void disabledPeriodic() {
    }

    /**
     * This function is called once when test mode is enabled.
     */
    @Override
public void testInit() {
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
public void testPeriodic() {
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
public void simulationPeriodic() {
    }

    publi
}
