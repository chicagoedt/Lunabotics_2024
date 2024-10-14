function setMode(mode) {
    fetch(`/set_mode/${mode}`, { method: 'POST' })
        .then(response => response.json())
        .then(data => {
            alert(`Robot set to ${data.mode} mode`);
            toggleMenu();
        })
        .catch(error => console.error('Error:', error));
}
