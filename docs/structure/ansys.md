# ANSYS Lectures

## Video Lectures with Clickable Timestamps

This page contains video lectures from YouTube. Click any timestamp to jump directly to that section in the video!

---

### Example: Introduction to ANSYS

<div class="video-container">
  <iframe id="ansys-intro-video" width="100%" height="600" src="https://www.youtube.com/embed/dQw4w9WgXcQ?enablejsapi=1" allow="autoplay" allowfullscreen></iframe>
</div>

**Click any timestamp to jump to that section:**

- [00:00 Introduction](javascript:seekYouTubeVideo('ansys-intro-video', 0))
- [00:50 Never Gonna](javascript:seekYouTubeVideo('ansys-intro-video', 50))
- [02:30 Give you up](javascript:seekYouTubeVideo('ansys-intro-video', 150))

---

### Example: Advanced Simulation

<div class="video-container">
  <iframe id="advanced-sim-video" width="100%" height="600" src="https://www.youtube.com/embed/jNQXAC9IVRw?enablejsapi=1" allow="autoplay" allowfullscreen></iframe>
</div>

**Click any timestamp to jump to that section:**

- [00:00 Overview](javascript:seekYouTubeVideo('advanced-sim-video', 0))
- [05:20 Setting Up Parameters](javascript:seekYouTubeVideo('advanced-sim-video', 320))
- [12:30 Material Properties](javascript:seekYouTubeVideo('advanced-sim-video', 750))
- [18:45 Running Analysis](javascript:seekYouTubeVideo('advanced-sim-video', 1125))
- [25:15 Results Discussion](javascript:seekYouTubeVideo('advanced-sim-video', 1515))

---

## How to Add Your Videos

### Step 1: Prepare Your YouTube Video
1. Upload your video to YouTube
2. Set video visibility to **Public** or **Unlisted** (so team can access it)
3. Copy the video URL

### Step 2: Extract the Video ID
From the YouTube URL: `https://www.youtube.com/watch?v=XXXXXXXXXX`

Copy the part after `v=` → this is your **VIDEO_ID**

⚠️ **Important:** Always add `?enablejsapi=1` to the YouTube embed URL for smooth timestamp seeking!

### Step 3: Add to Documentation
1. Copy one of the video section templates above
2. Replace `YOUR_VIDEO_ID` with your actual YouTube ID
3. Change the iframe ID to something unique (e.g., `my-video-1`)
4. Update timestamps with your own times (in seconds)
5. Update the timestamp links

### Step 4: Calculate Timestamp Times (in seconds)
Convert your timestamps to seconds:
- `00:00` = 0 seconds
- `02:30` = 150 seconds (2 × 60 + 30)
- `15:45` = 945 seconds (15 × 60 + 45)
- `1:30:45` = 5445 seconds (1 × 3600 + 30 × 60 + 45)
