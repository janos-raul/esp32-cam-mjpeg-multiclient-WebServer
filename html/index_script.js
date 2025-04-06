document.addEventListener('DOMContentLoaded', () => {
    const baseHost = document.location.origin;
    const streamUrl = `${baseHost}/stream`;

    const videoContainer = document.getElementById('videoContainer'); // The container for the video
    const mjpegStreamImg = document.createElement('img'); // Dynamically create an <img> element

    // Set the class for the <img> element
    mjpegStreamImg.className = 'mjpeg-stream';
    mjpegStreamImg.src = streamUrl;

    // Set dimensions to scale responsively while maintaining aspect ratio
    const aspectRatio = 640 / 480; // VGA aspect ratio (width / height)

    // Adjust dynamically based on the container's width
    function adjustStreamSize() {
        const containerWidth = videoContainer.offsetWidth;
        const calculatedHeight = containerWidth / aspectRatio; // Maintain aspect ratio

        mjpegStreamImg.style.width = `${containerWidth}px`;
        mjpegStreamImg.style.height = `${calculatedHeight}px`;
        videoContainer.style.height = `${calculatedHeight}px`;
    }

    // Initial adjustment
    adjustStreamSize();

    // Re-adjust on window resize
    window.addEventListener('resize', adjustStreamSize);

    // Add the <img> element to the video container
    videoContainer.appendChild(mjpegStreamImg);

    // Create the date and time element
    const dateTimeOverlay = document.createElement('div');
    dateTimeOverlay.className = 'date-time-overlay';
    videoContainer.appendChild(dateTimeOverlay);

    // Update the date and time dynamically
    setInterval(() => {
        const now = new Date();
        const formattedTime = now.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
        const formattedDate = now.toLocaleDateString();
        dateTimeOverlay.innerText = `${formattedDate} ${formattedTime}`;
    }, 1000);

    // Create the text element
    const overlayText = document.createElement('div');
    overlayText.className = 'overlay-text';
    overlayText.innerText = 'ESP32-CAM live video stream'; // Replace with your desired text
    videoContainer.appendChild(overlayText);

    // Add CSS styles for the overlays
    const style = document.createElement('style');
    style.innerHTML = `
        #videoContainer {
            position: relative; /* Make container the positioning context */
            max-width: 100%; /* Ensure container doesn't exceed screen width */
            margin: 0 auto; /* Center the video container */
	    max-width: 800px; /* Set a maximum width for desktop browsers */
        }
        .overlay-text {
            position: absolute;
            top: 10px;
            left: 10px;
            color: white;
            background-color: rgba(0, 0, 0, 0.5);
            padding: 5px 10px;
            font-size: 16px;
            font-family: Arial, sans-serif;
            border-radius: 5px;
        }
        .date-time-overlay {
            position: absolute;
            bottom: 10px;
            right: 10px;
            color: white;
            background-color: rgba(0, 0, 0, 0.5);
            padding: 5px 10px;
            font-size: 12px;
            font-family: Arial, sans-serif;
            border-radius: 5px;
        }
    `;
    document.head.appendChild(style);
});
