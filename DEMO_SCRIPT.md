# Demo video script

A ~3-minute video for LinkedIn / resume / portfolio. Hiring managers watch the first 30 seconds; nail that.

---

## Pre-production checklist (before you hit record)

- [ ] Device powered, flashed with `fw=0.7.0`, on Wi-Fi
- [ ] HiveMQ web client open in browser tab, **connected**, subscribed to:
  - `edgecam/<id>/telemetry`
  - `edgecam/<id>/detections`
  - `edgecam/<id>/cmd`
- [ ] VS Code monitor showing serial output, font bumped to ~16pt for readability
- [ ] Browser tab open at `http://<board-ip>/` (the snapshot page)
- [ ] Python HTTP server running on PC, serving a `v0.7.1.bin` (so OTA actually works mid-video)
- [ ] Desk lit decently — even ambient light, no harsh shadows on the camera lens
- [ ] Phone with a downloaded face photo to point at the camera (so you don't have to use your own)
- [ ] Mic test: AirPods, phone earbuds, or even built-in laptop mic — anything cleaner than a fan

---

## Equipment

- **Recording:** OBS Studio (free, screen capture). Or QuickTime (Mac), Game Bar (Windows 11), or the built-in screen recorder on any phone pointed at the monitor.
- **Editing:** DaVinci Resolve (free, overkill), or Clipchamp / iMovie / Shotcut. For a 3-min cut, anything with split + trim is enough.
- **Voiceover (optional):** record after editing the visuals, using your phone's voice memo app.

---

## Layout for screen recording

Two-pane split: VS Code monitor on the left (logs are the proof), browser on the right (HiveMQ + snapshot page, switching tabs as needed). 1080p is enough.

If you can't do split-pane, alt-tab between them — but cut to whichever pane is doing something.

---

## Script (3:00 total)

### 0:00 – 0:15 · Hook

**Visual:** Title card on black: `Edge-AI Vision Node · ESP32-S3 · On-device face detection` with your name and the GitHub URL underneath.

**Voiceover:**
> "This is an ESP32-S3 running an INT8 neural network on-device. It detects faces in 185 milliseconds and ships detection events to the cloud over TLS-encrypted MQTT. Let me show you what it does."

### 0:15 – 0:35 · Boot + connect

**Visual:** Cut to serial monitor. Show the boot sequence scrolling — PSRAM, partition table, Wi-Fi IP, TLS handshake, MQTT CONNECTED. Optional: subtle zoom on the `fw=0.7.0` line.

**Voiceover:**
> "Custom 8 MB partition table with dual OTA slots. PSRAM enumerated, camera detected over SCCB, Wi-Fi associated. TLS handshake validated against the Mozilla CA bundle. End-to-end from power-on to MQTT-TLS connected — about four seconds."

### 0:35 – 1:00 · Edge inference

**Visual:** Switch to HiveMQ subscribed view. Hold up the phone with face photo in front of the camera (you'll see the monitor or your hand). Cut to serial: `DET pub #1, #2, #3 score=0.87 ...`. Cut to HiveMQ: messages arriving in real time.

**Voiceover:**
> "Show a face — the model triggers at point eight seven confidence. Each detection becomes a JSON event with bounding box, score, frame index, and firmware version. Detections are rate-limited at the producer, capped at one per second, so the broker never sees a runaway device."

### 1:00 – 1:20 · Live HTTP snapshot

**Visual:** Switch to browser at `http://<board-ip>/`. The auto-refreshing snapshot shows the face photo. Hold the camera at a different angle, then a different photo.

**Voiceover:**
> "A built-in HTTP server software-encodes RGB565 frames to JPEG on demand. No video stream — too expensive while inference is running — but a snapshot is one click away. Useful for spot-checking what the model is seeing."

### 1:20 – 1:50 · Offline buffering

**Visual:** Open `main\secrets.h` in VS Code, change the Wi-Fi password to garbage, save. Cut to a "reflashing..." beat (or just say it'll lose Wi-Fi). Then show serial logs of `DET buffered #N (mqtt down)`. Then fix the password, reflash. Cut to serial: `spawning replay for N events` → `sd: replayed N events, file purged`. Cut to HiveMQ: events arriving in a tight burst.

**Voiceover:**
> "Drop the Wi-Fi. The device keeps detecting and writes events to an SD card. When the network heals, a replay task drains the buffer at twenty hertz and the cloud gets every event that happened during the outage. Zero events lost."

### 1:50 – 2:30 · Over-the-air firmware update

**Visual:** Cut to HiveMQ Send panel. Type `ota http://<host>:8000/v0.7.1.bin`. Send. Cut to serial: OTA download progress 10%, 20%, ..., 100%. Reboot. Show new `fw=0.7.1` line. Optionally zoom on `running partition: ota_1` to highlight the slot swap.

**Voiceover:**
> "Push a single MQTT command. The device downloads the new firmware over HTTPS — same Mozilla CA bundle reused — writes it to the inactive OTA slot, verifies the SHA, and reboots into the new image. About forty seconds for three megabytes. If the new image hangs, the task watchdog catches it and the bootloader rolls back. Bad firmware cannot brick this device."

### 2:30 – 2:50 · Recap

**Visual:** Static title card with three bullet points fading in:
- 185 ms on-device INT8 inference
- TLS-MQTT with SD-buffered offline resilience
- MQTT-triggered OTA with bootloader rollback

**Voiceover:**
> "Edge inference, secure transport, OTA with rollback, offline resilience. All in one firmware. Twelve hundred lines of C plus some C++ for the model wrapper. Three weekends of work."

### 2:50 – 3:00 · CTA

**Visual:** GitHub repo URL large on screen.

**Voiceover:**
> "Code, architecture docs, and the full engineering log are in the repo."

---

## Editing notes

- **Keep cuts tight.** No dead air. If a log line is what matters, cut to it and zoom on it.
- **Subtitles.** Hiring managers watch with sound off. Burn captions into the video — even auto-generated YouTube captions copied to your editor are fine.
- **Cursor highlighting.** OBS has a "show keys" overlay; use it for the moment you send the OTA command, so viewers know it's a real click.
- **No music** for technical demos. Voice + clicks + log scrolling is plenty. Music distracts.
- **Export 1080p, ~10 Mbps.** ~250 MB file. Uploads cleanly to LinkedIn (max 10 min, 5 GB).

---

## Where to host

| Platform | Notes |
|---|---|
| LinkedIn (native upload) | Best for recruiters; autoplays in feed; max 10 min. Upload directly, don't link YouTube. |
| YouTube unlisted | Embed in README, share via direct link. No randoms find it. |
| GitHub README embed | Use a thumbnail image linking to the video. GitHub doesn't autoplay. |
| Personal website | Self-host the MP4 if you have hosting. |

In your README, near the top:

```markdown
[![Demo](docs/demo_thumbnail.png)](https://www.youtube.com/watch?v=<id>)
```

The thumbnail image is just a 16:9 screenshot from the video with a play-button overlay. Photoshop / Figma / Canva can do this in 30 seconds.

---

## After uploading

Add this to your LinkedIn experience section as a media attachment:

> **Edge-AI Vision Node** — On-device face detection on ESP32-S3 with TLS-MQTT, OTA, SD buffering. [GitHub](https://github.com/<your-user>/edge-vision-node) · [Demo (3 min)](https://...)

Same on your resume — the URL goes in the Projects section.

---

*Total recording + editing time, first attempt: ~3 hours. Subsequent revisions: ~30 min each. Aim to ship a passable v1, not a perfect one.*
