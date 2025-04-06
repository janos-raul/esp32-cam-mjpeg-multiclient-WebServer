var baseHost = document.location.origin;
var streamUrl = `${baseHost}/stream`;
var jpgUrl = `${baseHost}/jpg`;
const view = document.getElementById('videoStream');
const viewContainer = document.getElementById('streamContainer');
const toggleStreamButton = document.getElementById('toggleStreamButton');
const getJpgButton = document.getElementById('getJpgButton');

const startStream = () => {
    view.src = streamUrl;
    viewContainer.style.display = 'flex';
    toggleStreamButton.innerHTML = 'Stop Video Stream';
    getJpgButton.disabled = true; // Disable "Save Picture" button
};

const stopStream = () => {
	// Remove the event listener to prevent unintended downloads
    view.onload = null;
    view.src = '';
    viewContainer.style.display = 'none';
    toggleStreamButton.innerHTML = 'Start Video Stream';
    getJpgButton.disabled = false; // Enable "Save Picture" button
};

const getJpg = async () => {
    try {
        const response = await fetch(jpgUrl);
        const blob = await response.blob();
        const blobUrl = URL.createObjectURL(blob);

        view.src = blobUrl; // Display the image
        viewContainer.style.display = 'flex';

        setTimeout(() => {
            downloadJpg(blobUrl); // Download the same image
        }, 500);
    } catch (error) {
        console.error('Error fetching image:', error);
    }
};

const downloadJpg = (url) => {
    fetch(url)
        .then(response => response.blob())
        .then(blob => {
            const link = document.createElement('a');
            const blobUrl = URL.createObjectURL(blob);

            // Get the current timestamp
            const now = new Date();
            const timestamp = now.toISOString().replace(/[:.-]/g, ''); // Format the timestamp

            // Append the timestamp to the filename
            link.href = blobUrl;
            link.download = `image_${timestamp}.jpg`;
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);

            // Revoke the Blob URL
            URL.revokeObjectURL(blobUrl);
        })
        .catch(error => console.error('Error downloading the image:', error));
};

toggleStreamButton.addEventListener('click', () => {
    if (viewContainer.style.display === 'none' || view.src !== streamUrl) {
        startStream();
    } else {
        stopStream();
    }
});

getJpgButton.addEventListener('click', () => {
    getJpg();
});

document.addEventListener('DOMContentLoaded', () => {
    fetch('/getSettings')
    .then(response => response.json())
    .then(settings => {
        document.getElementById('quality').value = settings.quality;
        document.getElementById('brightness').value = settings.brightness;
        document.getElementById('contrast').value = settings.contrast;
        document.getElementById('saturation').value = settings.saturation;
        document.getElementById('resolution').value = settings.resolution;
        document.getElementById('specialEffect').value = settings.specialEffect;
        document.getElementById('whiteBalance').value = settings.whiteBalance;
        document.getElementById('awbGain').value = settings.awbGain;
        document.getElementById('wbMode').value = settings.wbMode;
        document.getElementById('hmirror').value = settings.hmirror;
        document.getElementById('vflip').value = settings.vflip;
        document.getElementById('colorbar').value = settings.colorbar;
        document.getElementById('gammaCorrection').value = settings.gammaCorrection;
        document.getElementById('exposureControl').value = settings.exposureControl;
        document.getElementById('aec2').value = settings.aec2;
        document.getElementById('aeLevel').value = settings.aeLevel;
        document.getElementById('aecValue').value = settings.aecValue;
        document.getElementById('gainControl').value = settings.gainControl;
        document.getElementById('agcGain').value = settings.agcGain;
        document.getElementById('dcw').value = settings.dcw;
        document.getElementById('led').value = settings.led;
        document.getElementById('fps').value = settings.fps;
    })
    .catch(error => {
        console.error('Error loading settings:', error);
    });
});

document.getElementById('resetDefaultsButton').addEventListener('click', () => {
    // Resetting the values to their defaults
    document.getElementById('quality').value = '20';
    document.getElementById('fps').value = '14';
    document.getElementById('brightness').value = '0';
    document.getElementById('contrast').value = '0';
    document.getElementById('saturation').value = '0';
    document.getElementById('resolution').value = 'SVGA';
    document.getElementById('specialEffect').value = '0';
    document.getElementById('whiteBalance').value = '1';
    document.getElementById('awbGain').value = '1';
    document.getElementById('wbMode').value = '0';
    document.getElementById('hmirror').value = '0';
    document.getElementById('vflip').value = '0';
    document.getElementById('colorbar').value = '0';
    document.getElementById('gammaCorrection').value = '1';
    document.getElementById('exposureControl').value = '1';
    document.getElementById('aec2').value = '1';
    document.getElementById('aeLevel').value = '0';
    document.getElementById('aecValue').value = '10';
    document.getElementById('gainControl').value = '1';
    document.getElementById('agcGain').value = '15';
    document.getElementById('dcw').value = '1';
    document.getElementById('led').value = '0';

    // Trigger the "Save Settings" button click
    document.getElementById('saveSettingsButton').click();
});


/* ... existing code ... */

const saveButton = document.getElementById('saveSettingsButton');

const infoBar = document.getElementById('infoBar');
const showMessage = (message) => {
    infoBar.innerHTML = message;
    infoBar.style.display = 'block';
    setTimeout(() => {
        infoBar.style.display = 'none';
    }, 3000);
};

saveButton.addEventListener('click', () => {
    const settings = {
        quality: document.getElementById('quality').value,
        brightness: document.getElementById('brightness').value,
        contrast: document.getElementById('contrast').value,
        saturation: document.getElementById('saturation').value,
        resolution: document.getElementById('resolution').value,
        specialEffect: document.getElementById('specialEffect').value,
        whiteBalance: document.getElementById('whiteBalance').value,
        awbGain: document.getElementById('awbGain').value,
        wbMode: document.getElementById('wbMode').value,
        hmirror: document.getElementById('hmirror').value,
        vflip: document.getElementById('vflip').value,
        colorbar: document.getElementById('colorbar').value,
        gammaCorrection: document.getElementById('gammaCorrection').value,
        exposureControl: document.getElementById('exposureControl').value,
        aec2: document.getElementById('aec2').value,
        aeLevel: document.getElementById('aeLevel').value,
        aecValue: document.getElementById('aecValue').value,
        gainControl: document.getElementById('gainControl').value,
        agcGain: document.getElementById('agcGain').value,
        dcw: document.getElementById('dcw').value,
        fps: document.getElementById('fps').value,
	led: document.getElementById('led').value
    };

    fetch('/settings', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify(settings)
    })
    .then(response => response.json())
    .then(data => {
        console.log('Settings saved:', data);
        showMessage('Settings applied successfully');
    })
    .catch((error) => {
        console.error('Error:', error);
        showMessage('Error applying settings');
    });
});
