# FRC Team 3512's 2022 Robot

Source code for the 2022 comp robot: Daedalus

Source code also for the 2022 practice robot: Xiphos

- [FRC team 3512's 2022 robot](#frc-team-3512-s-2022-robot)
  * [Setup](#setup)
  * [C++ intellisense in Vim with clangd](#c---intellisense-in-vim-with-clangd)
  * [Documentation](#documentation)
  * [Build everything](#build-everything)
    + [Build (athena)](#build--athena-)
    + [Deploy](#deploy)
    + [roboRIO imaging](#roborio-imaging)
    + [Test](#test)
    + [Simulation GUI](#simulation-gui)
    + [API documentation](#api-documentation)
    + [Debugger](#debugger)
    + [Address sanitizer](#address-sanitizer)
    + [Thread sanitizer](#thread-sanitizer)
    + [Undefined behavior sanitizer](#undefined-behavior-sanitizer)
  * [Live logging](#live-logging)
    + [OutlineViewer](#outlineviewer)
    + [CSV](#csv)
    + [Glass](#glass)
  * [Simulation logging](#simulation-logging)
    + [Simulation GUI](simulation-gui-1)
    + [CSV](#csv-1)
  * [Autonomous mode selection](#autonomous-mode-selection)
  * [Game](#game)
  * [Unique features](#unique-features)
  * [Goals of the year](#goals-of-the-year)
  * [Roster](#roster)

## Setup

Install the relevant FRC toolchain for your platform (see
https://github.com/wpilibsuite/allwpilib/releases). Make sure the toolchain
is
placed in `~/wpilib/2022/roborio` (Linux) or
`C:\Users\Public\wpilib\2022\roborio` (Windows).

Install the following OS packages.

* gcc >= 7.3.0
* python >= 3.6
* scp (optional, for CSV logging)

Install the following python packages via `pip3 install --user package_name`.

* wpiformat (optional)
  * https://github.com/wpilibsuite/styleguide/blob/wpiformat/README.rst

We use the latest version of clang-format with wpiformat for C++ formatting.

## C++ intellisense in Vim with clangd

VSCode's intellisense is occasionally unreliable. Vim + clangd is an alternative
that provides better intellisense because it actually invokes a C++ compiler frontend. It also generates linting annotations via clang-tidy. Setup is as follows.

1. Install Node.js. You should have a `node` executable in PATH.
2. Install clangd, which comes with the [LLVM compiler](https://release.llvm.org/download.html)
3. Add  `Plugin` 'neoclide/coc.nvim'` to your `.vimrc`
4. In vim, run `:CocInstall coc-clangd`
5. Run `./gradlew intellisense`

The clangd indexer will start when a file is opened in Vim. See https://github.com/clangd/coc-clangd for troubleshooting steps if needed.

## Documentation

TODO: Write docs

## Build options

### Build everything

* `./gradlew build`

This runs a roboRIO and desktop build and runs the desktop tests. This may take a while, so more specific builds (see below) are recommended instead.

### Build (athena)

* `./gradlew buildAthena`

This runs a roboRIO build.

### Deploy

* `./gradlew deploy`

This runs a roboRIO build if needed, copies the resulting binary to a roboRIO at 10.35.12.2, and restarts it.

### Test

Unit tests are useful for ensuring parts of the robot code continue to work correctly after implementing new features or refactoring existing functionality.

* `./gradlew testDebug `

This runs a debug build of the robot code's unit tests from `src/test`.

### Simulation GUI

* `./gradlew simulateNative`

This runs a release build of the robot code in the simulation GUI.

### API documentation
* `./gradlew doxygen`

This command generates HTML documentation for the robot code from in-source Doxygen comments. The results are placed in `docs/html` folder with an `index.html` page as the root.

### Debugger

* `./buildscripts/debug_test.py`

This runs a debug build of the tests in GDB. Once the build completes and the debugger's prompt appears, enter `run` to start the robot program. It may take a while due to the debugger having to load a lot symbols. If the robot code crashes, enter `bt` to get a backtrace.

* `./buildscripts/debug_simulation.py`

This runs a debug desktop build of the robot code and simulation GUI in a debugger.

* `./buildscripts/gdb-test-ci.sh`

This runs a debug build of the tests in GDB in noninteractive mode. It will automatically run the program in the debugger and print a backtrace if it crashes. This is useful for debugging crashes in GitHub Actions.

### Address sanitizer

`./gradlew test -Pasan`

This runs a release build of the tests with the address sanitizer enabled.
The address sanitizer is useful for finding memory corruption and reads from uninitialized memory so they can be fixed.

### Thread sanitizer

`./gradlew test -Ptsan`

This runs a release build of the tests with the thread sanitizer enabled.
The thread sanitizer is useful for finding race conditions.

### Undefined behavior sanitizer

`./gradlew test -Pubsan`

This runs a release build of the tests with the undefined sanitizer enabled.

## Live logging

Logging can be viewed while the robot is running and in simulation.
**OutlineViewer**, **CSV**, and **Glass** are the main ways to view logs while the robot is running. ControllerBase supports two logging backends for high-throughput controller performance data: **CSV** and **Glass**.

### OutlineViewer

OutlineViewer is a WPIlib tool to view NetworkTables.

* Make sure to have FRC toolchain from the setup section.
* Open the tools directory `~/wpilib/2022/tools` and run `python3 ToolsUpdater.py`
* Open OutlineViewer by running `./gradlew OutlineViewer` and set the server location to 10.35.12.2. The default port will work.

### CSV

This backend writes CSV files to the roboRIO flash storage. After they are recorded, they can be retrieved with `tools/get_csvs.py` and displayed with `tools/plot_subsystems.py`.

### Glass

Glass is a WPILib tool that allows for pose visualization and networktable plotting/visualization while the robot is running. See the [Glass documentation](https://docs.wpilib.org/en/latest/docs/software/dashboards/glass/introduction.html)
for more details.

* Make sure to have the FRC toolchain from the setup section.
* Open the tools directory `~/wpilib/2022/tools` and run `python3 ToolsUpdater.py`.
* Open glass by running `./gradlew Glass` and set the server location to 10.35.12.2. The default port will work.

## Simulation logging.

Logs can be viewed in real time via NetworkTables in simulation GUI or offline via CSV processing.

### Simulation GUI

The simulation GUI is straightforward but can be read more about [here](https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-simulation/simulation-gui.html).

### CSV

After running the tests, the CSV files will be saved. The backend writes CSV files to `build/test-results/frcUserProgramTest`. To display the CSVs, run the following command:

```
./tools/plot_subsystems.py [regexp]
```
`plot_subsystems.py` will display CSVs whose filepaths match the optional regular expression `[regexp]`. It should be given filepath components after the `frcUserProgramTest` folder.

To show data for a specific subsystem, include its name in the regular expression.

```
./tools/plot_subsystem.py Flywheel
```

Specific states, inputs, or outputs can be viewed as well.

```
./tools/plot_subsystem.pu "Flywheel states"
```

Other examples:

```
./tools/plot_subsystem.py "IntakeTest/Deploy"
./tools/plot_subsystem.py "AutonomousTest/AutonomousTest/Run/Shoot Two/Drivetrain (States|Outputs)"
./tools/plot_subsystem.py "FlyweehlTest/ReachesGoal/Front Flywheel timing"
```

## Autonomous mode selection

Open shuffleboard and select the desired autonomous mode from the dropdown menu.
When the indicator next to the menu turns from a red X to a green checkmark, the robot has confirmed the selection.

See [this](https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html)
for details on how the robot side works.

## Game

The game for 2022 is called Rapid React, where teams are tasked with shooting cargo into either a lower or upper hub. This year there is the standard 15 second autonomous period. Teams earn points in this period for moving off of the tarmac and earn double the usual points for scoring in the hub. If the alliance can score 5 cargo during autonomous, then the number of cargo needed for the Tele-op ranking point is reduced from 20 to 18. Tele-op continues this gameplay while the robot is controlled through driver input and the cargo points are cut in half. Alliances can earn a ranking point for scoring a certain number of cargo in the hub, be it upper or lower. The number of cargo is dependent on how much cargo was scored during autonomous as stated previously. In Tele-op there is the addition to climb in the hangar. The hangar has four levels of bars the increase in height from the ground and are separated from each other. They cascade over and up towards the driverstation. Robots can traverse up these bars, gaining more points for the higher bar they reach, gaining enough points as an alliance grants a ranking point. Climbing is not prohibited to a certain time during the match, meaning there is no Endgame sequence.

## Unique features

This years robot's unique features include:

- Augmented state-space drivetrain controller
- Fourbar linkage intake/outtake
- Funnel
- Indexing conveyor
- Front and Back Flywheels run independently for variable shooting distances
- Raspberry Pi 3 w/ vision processing
- Passive arms that leave robot hanging on bar
- Active climber arms that can be actuated forward and back to traverse to highest rung

## Goals of the year
|Status|Goal|Additional Description|
|------|----|----------------------|
|Yes|Aim Drivetrain With Vision|Aim the drivetrain at the center of the vision target in order to shoot cargo in correctly.|
|Yes|Separate Flywheel State Space Controllers|Use separate state spaces controllers to run a front and back flywheel and different speeds for variable shooting.|
|Yes|Simulate Climber And Intake|Use the Mechanism2d widget to test climbers and intake before having a physical robot to test.|

## Roster
Mentors: Tyler Veness

Students: Matthew Santana (Lead), Adan Silva, Evelyn Mejia
