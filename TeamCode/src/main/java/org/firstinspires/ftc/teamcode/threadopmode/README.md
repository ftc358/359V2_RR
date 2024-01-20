[//]: # (#Credit to Nolan1234 on Github)

[//]: # ()
[//]: # ()
[//]: # (# FTC-Threaded-OpMode)

[//]: # (A library for FTC Java programming that makes multi threading easy.)

[//]: # (## How to install)

[//]: # (1. Download and extract this repository)

[//]: # (2. Drag the folder `threadopmode` into the `org.firstinspires.ftc.teamcode` package of your `TeamCode` module)

[//]: # (## How to use)

[//]: # (1. Add the namespace to the top of your program)

[//]: # (```java)

[//]: # (import org.firstinspires.ftc.teamcode.threadopmode.*;)

[//]: # (```)

[//]: # (2. Have your class extend `ThreadOpMode` rather than `OpMode`)

[//]: # (```java)

[//]: # (public class ExampleOpMode extends ThreadOpMode {)

[//]: # (  ...)

[//]: # (})

[//]: # (```)

[//]: # (3. Override the `mainInit` and `mainLoop` methods)

[//]: # (```java)

[//]: # (@Override)

[//]: # (public void mainInit&#40;&#41; {)

[//]: # ()
[//]: # (})

[//]: # ()
[//]: # (@Override)

[//]: # (public void mainLoop&#40;&#41; {)

[//]: # ()
[//]: # (})

[//]: # (```)

[//]: # (3. Add regular `init` code to `mainInit`)

[//]: # (4. Create a new thread in `mainInit` using the following template)

[//]: # (```java)

[//]: # (registerThread&#40;new TaskThread&#40;new TaskThread.Actions&#40;&#41; {)

[//]: # (    @Override)

[//]: # (    public void loop&#40;&#41; {)

[//]: # (        //The loop method should contain loop code)

[//]: # (    })

[//]: # (}&#41;&#41;;)

[//]: # (```)

[//]: # (5. Repeat this for as many threads as you want to spawn)

[//]: # (6. &#40;Optional&#41; add code to the main thread in `mainLoop`)

[//]: # (```java)

[//]: # (@Override)

[//]: # (public void mainLoop&#40;&#41; {)

[//]: # (    //Anything you want to periodically run in the MAIN thread goes here)

[//]: # (})

[//]: # (```)
