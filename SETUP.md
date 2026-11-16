# Setup: ITKAN FutureGen robot code (BIOBUZZ 2026–27)

Read this before writing your own TeleOp or Pedro auton. It assumes a Mac; Windows is the same except for the adb path.

## 1. One-time setup on your laptop

1. Install **Android Studio** (Narwhal 3 Feature Drop or newer, per README.md). It comes with the Java it needs.
2. Clone this repo, then in Android Studio: **File > Open** and pick the cloned folder (the one with `settings.gradle`).
3. Let the first **Gradle sync** finish. **This needs normal internet** (not the robot's Wi-Fi): it downloads Pedro Pathing and Ivy.
   After one successful sync, builds also work offline.
4. Android Studio creates `local.properties` for you. It's machine-specific and gitignored, so never commit it.

If the sync fails:
- "Unknown host" / "could not resolve": you're on the robot's Wi-Fi or offline. Switch to internet and sync again.
- Errors mentioning the Gradle user home or a JDK folder: **Settings > Build, Execution, Deployment > Build Tools > Gradle**. Leave "Gradle user home" empty and set "Gradle JDK" to the bundled one (JetBrains Runtime / "jbr").

## 2. What you don't need to do

- **No Pedro tuning.** The tuned values are in `TeamCode/.../pedro/Constants.java` and come with the repo. Tuning belongs to the robot, not the person. Re-tune only if the drivetrain, wheels or odometry pods change, and tell the team before you do.
- **No Driver Hub configuration.** The robot configuration is already on the robot and other people rely on it, so don't edit it. Device names are in [PORTS.md](PORTS.md).

## 3. Adding your own program

- Put your file in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
- Give it a **unique class name** (the class name must match the file name) and a **unique Driver Hub name** in `@TeleOp(name = ...)` / `@Autonomous(name = ...)`.
- For a Pedro auton: start from a copy of `VisualizerPathOnly.java` (or your own visualizer export). Every Pedro auton gets the robot from `Constants.create(hardwareMap)`, and the visualizer export forgets `import org.firstinspires.ftc.teamcode.pedro.Constants;`, so add it.
- **Don't edit** `pedro/Constants.java`, `PORTS.md`, or other people's files without asking. Drive motor directions are set in both `MainTeleOp.java` and `pedro/Constants.java` and must match.
- The route the team uses is saved in `visualizer/biobuzz-red-full.pp.json`. Open it at https://visualizer.pedropathing.com with **File > Open from computer**.

## 4. Putting code on the robot

### Option A: USB-C cable (most reliable)

Plug the laptop into the Control Hub's USB-C port, pick the Control Hub in Android Studio's device menu, and press Run ▶. No commands needed.

### Option B: Wi-Fi with adb

`adb` (Android Debug Bridge) is the tool Android Studio uses to talk to the Control Hub. Over Wi-Fi you have to connect it yourself.

**One-time: make `adb` a short command.** It's installed with Android Studio but isn't on your PATH. On a Mac, in Terminal:

```
echo 'export PATH="$PATH:$HOME/Library/Android/sdk/platform-tools"' >> ~/.zshrc
source ~/.zshrc
adb version
```

If `adb version` prints a version number, it worked. (Windows: adb is at `%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe`.)
Without this step, type the full path instead of `adb`: `~/Library/Android/sdk/platform-tools/adb`.

**Every time you deploy over Wi-Fi:**

1. Turn the robot on and wait for the Control Hub's light to turn green (about 20 seconds).
2. Connect the laptop to the robot's Wi-Fi network (the Control Hub's name). The laptop has no internet while it's connected.
3. In Terminal:
   ```
   adb kill-server
   adb connect 192.168.43.1:5555
   adb devices
   ```
   - `adb kill-server` clears any old, stuck connection.
   - `adb connect` connects to the Control Hub. `192.168.43.1` is the Control Hub's address on its own Wi-Fi, and `5555` is the adb port.
   - `adb devices` should list `192.168.43.1:5555    device`. The word **device** means it's ready.
4. In Android Studio, pick the Control Hub in the device menu. Don't pick "Medium Phone API": that's a phone emulator on your laptop, not the robot. Then press Run ▶.
5. When you're done, run `adb disconnect` and switch back to your normal Wi-Fi.

**If something goes wrong:**

| You see | Do this |
|---|---|
| `already connected to 192.168.43.1:5555`, but Android Studio can't deploy | The old connection is stale. Run `adb kill-server`, then `adb connect 192.168.43.1:5555` again. |
| `offline` in `adb devices`, or "device offline" in Android Studio | Same as above. If it's still offline, power-cycle the robot, wait for green, and connect again. |
| `protocol fault` | Run `adb kill-server`, power-cycle the robot, and connect again. Or use the cable. |
| `failed to connect` or `connection refused` | Check that the laptop is on the robot's Wi-Fi and the robot has finished booting. |
| `zsh: command not found: adb` | Do the one-time PATH step above, or type the full path. |
| Gradle sync fails while on the robot's Wi-Fi | That's expected: no internet. Sync on normal Wi-Fi first. Builds work offline after that. |

## 5. Sharing your work

Work on your own git branch (`git switch -c your-name/what-you-are-doing`), commit only your own files, and open a pull request. Never commit `local.properties` or `build/` folders.

## Files at a glance

| File | What it is |
|---|---|
| `TeamCode/.../MainTeleOp.java` | YusufTele, the driver program |
| `TeamCode/.../BioBuzzVisualizerAuton.java` (+ `Blue`) | Competition auton: route + intake + flower bar + early park |
| `TeamCode/.../VisualizerPathOnly.java` | Visualizer export, driving only, with TODOs for adding mechanisms |
| `TeamCode/.../EncoderCheck.java` | Shows encoder counts, for filling in PORTS.md |
| `TeamCode/.../pedro/Constants.java` | Tuned Pedro settings. Don't edit casually |
| `TeamCode/.../pedro/Tuning.java`, `procedures/` | Pedro's tuning programs |
| `PORTS.md` | Which device is plugged in where, and motor directions |
| `visualizer/biobuzz-red-full.pp.json` | The route, to reopen in the visualizer |
