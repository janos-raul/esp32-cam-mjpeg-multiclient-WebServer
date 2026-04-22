document.addEventListener('DOMContentLoaded', () => {
    const baseHost = document.location.origin;
    const streamUrl = `${baseHost}/stream`;

    const videoContainer = document.getElementById('videoContainer'); // The container for the video
    const mjpegStreamImg = document.createElement('img'); // Dynamically create an <img> element

    // Set the class for the <img> element
    mjpegStreamImg.className = 'mjpeg-stream';
    mjpegStreamImg.src = streamUrl;

    // Set dimensions to scale responsively while maintaining aspect ratio
    const aspectRatio = 4 / 3; // 4:3 aspect ratio (standard camera)

    // Let CSS handle the sizing with object-fit: contain
    // No need to manually set dimensions

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
            position: relative;
            width: 100%;
            max-width: 800px;
            height: 600px;
            margin: 0 auto;
            display: flex;
            align-items: center;
            justify-content: center;
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
