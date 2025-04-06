function areFieldsFilled() {
    const ssid = document.getElementById('ssid').value.trim();
    const password = document.getElementById('password').value.trim();
    const hostname = document.getElementById('hostname').value.trim();

    return ssid && password && hostname;
}

document.getElementById('saveButton').addEventListener('click', () => {
    const ssid = document.getElementById('ssid').value.trim();
    const password = document.getElementById('password').value.trim();
    const hostname = document.getElementById('hostname').value.trim();

    if (!ssid || !password || !hostname) {
        alert('Please fill out all fields.');
        return;
    }

    const settings = {
        ssid,
        password,
        hostname
    };

    fetch('/saveSettings', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify(settings)
    }).then(response => response.json())
      .then(data => alert(data.message));
});

document.getElementById('rebootButton').addEventListener('click', () => {
    if (!areFieldsFilled()) {
        alert('Please fill out all fields before rebooting.');
        return;
    }
    
    fetch('/reboot', {
        method: 'POST'
    }).then(response => response.json())
      .then(data => alert(data.message));
});

document.getElementById('rebootApButton').addEventListener('click', () => {
    fetch('/rebootAp', {
        method: 'POST'
    }).then(response => response.json())
      .then(data => {
        document.getElementById('apInfo').style.display = 'block';
        alert(data.message);
    });
});
