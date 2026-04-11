/**
 * YouTube Video Timestamp Seeker
 * Allows clickable timestamps to seek to specific times in embedded YouTube videos
 * Uses YouTube's IFrame API without reloading the video
 */

// Store YT API ready state
let youtubeApiReady = false;
let players = {};

// Load YouTube IFrame API
function loadYoutubeAPI() {
  if (!window.YT) {
    const tag = document.createElement('script');
    tag.src = 'https://www.youtube.com/iframe_api';
    const firstScriptTag = document.getElementsByTagName('script')[0];
    firstScriptTag.parentNode.insertBefore(tag, firstScriptTag);
  }
}

// Called by YouTube API when ready
window.onYouTubeIframeAPIReady = function() {
  youtubeApiReady = true;
  initializeYouTubePlayers();
};

// Initialize all YouTube players
function initializeYouTubePlayers() {
  const iframes = document.querySelectorAll('iframe[id][src*="youtube.com"]');
  iframes.forEach(iframe => {
    if (!players[iframe.id]) {
      players[iframe.id] = new YT.Player(iframe.id, {
        events: {}
      });
    }
  });
}

// Seek to timestamp
function seekYouTubeVideo(iframeId, seconds) {
  if (youtubeApiReady && players[iframeId]) {
    players[iframeId].seekTo(seconds, true);
  } else {
    // Fallback if API isn't ready yet
    setTimeout(() => seekYouTubeVideo(iframeId, seconds), 500);
  }
}

// Load API when DOM is ready
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', loadYoutubeAPI);
} else {
  loadYoutubeAPI();
}



