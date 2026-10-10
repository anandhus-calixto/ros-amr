# Robot network setup (onboard router, no USB WiFi dongle)

Set up on 2026-10-09. This explains how the robot connects to the office
network, how each part is configured, and how to fix it if something breaks.

---

## 1. The short version

The robot used to join the office WiFi with a small **USB WiFi dongle**. That
dongle was unreliable, so it was removed.

Now the robot has an **industrial router (USR-G806w) mounted inside it**. The
router joins the office WiFi, and the robot's computer (the i.MX8MP board)
talks to the router through a **short Ethernet cable**.

From your laptop, nothing changed:

| What you want            | How                                   |
|--------------------------|---------------------------------------|
| Log in to the robot      | `ssh imx8mp` (same as before)         |
| Open the web HMI         | `http://192.168.1.62:8080`            |
| See ROS2 topics          | `ros2 topic list` on the laptop       |
| Open the router settings | `http://192.168.1.250` in a browser   |

Your laptop only needs to be connected to the office WiFi **Calixto_Airtel**.

---

## 2. What is physically inside the robot

```
                         office WiFi "Calixto_Airtel"
                                   ^
                                   |  (WiFi, 2.4 GHz)
                                   |
   +-------------------------------|------------------------------+
   |  ROBOT                        |                              |
   |                    +----------+----------+                   |
   |                    | USR-G806w router    |                   |
   |                    +----------+----------+                   |
   |                               | Ethernet                     |
   |                        +------+------+                       |
   |                        |   switch    |                       |
   |                        +--+-------+--+                       |
   |                 Ethernet  |       |  Ethernet                |
   |               +-----------+       +-----------+              |
   |               |                               |              |
   |      +--------+--------+             +--------+--------+     |
   |      | i.MX8MP board   |             | LiDAR           |     |
   |      | port "end1"     |             |                 |     |
   |      +-----------------+             +-----------------+     |
   +--------------------------------------------------------------+
```

Why there is a switch: the i.MX8MP board has **only one Ethernet port**
(`end1`). Both the LiDAR and the router need a wire to it. The switch lets
both share that one port.

---

## 3. Every address, and what it belongs to

The office WiFi network uses addresses that start with `192.168.1.`.
The LiDAR uses its own small network, with addresses that start with
`192.168.31.`. Both run over the same cable into the robot.

| Address          | Belongs to                         | Used for                              |
|------------------|------------------------------------|---------------------------------------|
| `192.168.1.1`    | The office router (Calixto_Airtel) | The way out to the internet           |
| `192.168.1.62`   | **The robot (i.MX8MP), port end1** | SSH, web HMI, ROS2 topics             |
| `192.168.1.250`  | The USR router inside the robot    | Opening its settings page (LuCI)      |
| `192.168.1.70`   | The laptop (example, can change)   | Your machine on the office WiFi       |
| `192.168.31.7`   | **The robot (i.MX8MP), port end1** | Talking to the LiDAR                  |
| `192.168.31.19`  | The LiDAR                          | Sends scan data to the robot          |

Note that the robot has **two addresses on the same port** (`192.168.1.62`
and `192.168.31.7`). That is normal on Linux: one cable, two addresses, two
purposes.

`192.168.1.62` is the same address the USB dongle used to have. It was kept on
purpose, so `ssh imx8mp`, the HMI link and `launch/rviz_remote.launch.py` all
keep working without any change.

---

## 4. How the USR router is set up

The router is the important part. It works as a **bridge**: it joins the
office WiFi and passes everything straight through to its Ethernet ports, as if
the robot were plugged directly into the office router. There is no separate
private network and no address translation (NAT) in between. That is why ROS2
topics are visible from the laptop.

### 4.1 How to open the router's settings page

- Connect your laptop to **Calixto_Airtel**.
- Open `http://192.168.1.250` in a browser.
- The page is called **LuCI** and the device shows as **USR-G806w**.

### 4.2 Setting 1: join the office WiFi (WLAN STA Settings page)

This is the vendor's own page, not under Network > Wireless.

| Field     | Value              |
|-----------|--------------------|
| Enable    | ticked             |
| SSID      | `Calixto_Airtel`   |
| Encryption| `mixed-psk`        |
| Key       | the office WiFi password |
| network   | **`lan`**          |
| Force update of LAN IP address | off |

`network = lan` is what turns the router into a bridge. It joins the WiFi
connection with the Ethernet ports into one network.

**Important:** the router's radio is **2.4 GHz only**. It cannot join
`Calixto_Airtel_5GHz`. It must use `Calixto_Airtel` (the 2.4 GHz name).

### 4.3 Setting 2: the router's own address (Network > Interfaces > LAN > Edit)

| Field          | Value              |
|----------------|--------------------|
| Protocol       | static             |
| IPv4 address   | `192.168.1.250`    |
| IPv4 netmask   | `255.255.255.0`    |
| IPv4 gateway   | `192.168.1.1`      |
| DNS            | `192.168.1.1`      |
| DHCP Server    | **"Ignore interface" ticked** (router hands out no addresses) |

Why the router's own DHCP is switched off: the office router already hands out
addresses on this network. Two DHCP servers on one network fight each other.

After the change, the LAN overview should show:

```
LAN (eth1.1  Master "calixto-amr"  Client "Calixto_Airtel")
IPv4: 192.168.1.250/24
```

### 4.4 The router's own WiFi "calixto-amr" (optional)

The router also broadcasts its own WiFi called `calixto-amr`. It is part of the
same bridge, so a device that joins `calixto-amr` also lands on the office
network. The robot does **not** use it. It can stay on or be turned off. Its
password is set in LuCI and is not written here on purpose (this file is in
git).

### 4.5 A LuCI trap to know about

When you change a network setting and press **Save & Apply**, LuCI applies it
for a short time and waits for you to confirm. If you don't confirm in time,
**it quietly undoes the change**.

- If a confirm box appears, click it.
- If the change moves the router to a new address (like changing the LAN IP),
  your browser can't reach it to confirm. Choose **"Apply unchecked"** instead,
  then open the router at its new address.

---

## 5. How the robot (i.MX8MP) is set up

The robot's port `end1` is **not** managed by NetworkManager. Its addresses
are set at boot by the stack supervisor (`docker/amr-stack-supervisor.sh`,
see 5.1), and set again, harmlessly, by `docker/run_full_stack.sh` every time
the stack starts (so a stack started by hand also gets them). Both use the
same values; if you ever change one, change the other:

```
sudo ip addr add 192.168.31.7/24 dev end1        # LiDAR side
sudo ip addr add 192.168.1.62/24 dev end1        # office network side
sudo ip route replace default via 192.168.1.1 dev end1   # internet
```

These lines run **before any ROS node starts**. That order matters: a ROS2
node only tells other machines about the addresses the robot had at the moment
the node started. If a node starts before `192.168.1.62` exists, the laptop
can't see its topics until the stack is restarted.

The addresses can be changed with extra arguments to the script (4th =
office address, 5th = gateway), but the defaults above are what is used.

### 5.1 Clock sync before the stack starts

The board's clock battery (RTC supercap) is missing, so after every power-off
it boots with an old time and only corrects it once it reaches the internet.
If the clock jumps forward **after** the ROS stack has started, the motor
control loop (`ros2_control_node`) thinks it is hours behind and runs about
10x too fast to catch up. On 2026-10-10 this was seen as `/joint_states` and
`/mcu_status` at ~200 Hz instead of 20 Hz, the web HMI server at 80% CPU with
no browser open, and a sluggish initial pose.

So the stack supervisor (`docker/amr-stack-supervisor.sh`, run by
`amr-stack.service`) now does this at boot, in order:

1. Sets the `end1` addresses and the default route (same values as above).
2. Restarts the time-sync service and waits up to 60 s for the clock to sync.
   Without internet it gives up after 60 s and starts the stack anyway.
3. Starts the ROS stack.

While running, every 5 s it also checks whether the wall clock has jumped by
more than 30 s (for example, internet arrived late). If so, it restarts the
stack once, the same way it already does when a core process dies.
Navigation has to be started again after such a restart.

Once the supercap is fitted these safeguards cost nothing: the wait finishes
at once and the jump check never fires.

To see what happened at the last start: `journalctl -u amr-stack | grep -E "clock|starting|restart"`.
To check the loop rate: `ros2 topic hz /joint_states` should say about 20.

### 5.2 How to check the robot's addresses

```
ip -4 addr show end1
```

You should see both `192.168.31.7/24` and `192.168.1.62/24`.

**Do not trust `ifconfig` for this.** It is an old tool that shows only one
address per port, so it shows `192.168.31.7` and hides `192.168.1.62`. Nothing
is wrong when that happens; use `ip` instead. (`ip -br addr` gives a short
one-line view of every port.)

---

## 6. How the laptop is set up

Nothing special:

- Connect to WiFi **Calixto_Airtel**.
- `~/.ssh/config` has the host `imx8mp` pointing to `192.168.1.62` (user `root`).
- `ros2 topic list` shows the robot's topics. On 2026-10-09 this was tested
  over WiFi with plain default discovery: 36 topics, including `/scan`, `/tf`
  and `/odometry/filtered`.
- `launch/rviz_remote.launch.py` also still works: it points at
  `192.168.1.62` too.

---

## 7. Serial console (when the network is down)

If the robot can't be reached over the network, use the serial console. It
needs no network at all.

- A USB-to-serial adapter on the laptop shows up as `/dev/ttyUSB0`.
- Wiring: adapter TX to board RX, adapter RX to board TX, GND to GND.
- Settings: **115200** baud, 8 data bits, no parity, 1 stop bit.
- Connect with: `screen /dev/ttyUSB0 115200` (exit with `Ctrl-A` then `K`)
  or `minicom -D /dev/ttyUSB0 -b 115200`.
- Press Enter. You get a root prompt on the robot.

**Don't be confused by the prompt:** the robot's hostname is
`calixtosystems-HP-Notebook`, which looks like a laptop name. It really is the
robot. To be sure, run `uname -m`: the robot answers `aarch64`, a normal laptop
answers `x86_64`.

---

## 8. If something goes wrong

**The robot does not answer `ssh imx8mp` or `ping 192.168.1.62`**
1. Open the router page at `http://192.168.1.250`. If it doesn't open, the
   router is off, or it lost the WiFi, or its settings were reset (see below).
2. On the router's WLAN STA page, check it is connected to `Calixto_Airtel`
   (a signal value like -50 dBm is good).
3. Use the serial console (section 7) and run `ip -4 addr show end1`. If
   `192.168.1.62` is missing, the stack didn't start: `systemctl status amr-stack`.

**SSH works but the laptop sees only `/parameter_events` and `/rosout`**
- Either the ROS stack on the robot is not running, or it started before the
  office address existed. Restart it: `systemctl restart amr-stack`, wait about
  40 seconds, try again.
- Also make sure the laptop uses the same ROS domain (by default
  `ROS_DOMAIN_ID` is unset, which means 0, on both).

**The LiDAR stopped (`/scan` missing)**
- On the robot: `ping 192.168.31.19`. If it fails, check the LiDAR's cable and
  the switch's power. The LiDAR must be on the switch, not plugged elsewhere.

**The robot has no internet**
- On the robot: `ip route | grep default` should show `via 192.168.1.1 dev end1`.

**The router was reset to factory settings**
- After a reset its own address goes back to `192.168.1.1`, the **same as the
  office router**, and it starts handing out addresses again. That breaks
  things. To reach it, plug a laptop straight into one of its LAN ports, open
  `http://192.168.1.1`, then redo section 4 (both settings).
- A config backup from **before** this setup exists
  (`backup-USR-G806w-2025-10-13.tar.gz`, on the laptop in `~/Downloads`). It is
  the **old** setup (private `192.168.31.x` network), not this one. Restoring
  it will not give you this setup. Take a fresh backup of the working setup:
  LuCI > System > Backup / Flash Firmware > "Generate archive".

---

## 9. Things to know

- `192.168.1.62` and `192.168.1.250` were picked because nothing answered on
  them on 2026-10-09. They are set by hand, so the office router does not
  know about them. If the office router ever hands one of them to another
  device, two devices will fight over the address. The safe fix is to reserve
  both addresses in the office router's settings, if someone has access to it.
- The board's Ethernet MAC address on `end1` was different on several boots
  (it seems to be random each boot). That doesn't matter here because the
  addresses are set by hand, but it would matter for any DHCP reservation
  based on the MAC.
- The USR router is still listed as having 4G and wired-WAN uplinks
  (`WAN_4G`, `WAN_WIRED`). They are unused. The only uplink is the WiFi
  connection to `Calixto_Airtel`.
