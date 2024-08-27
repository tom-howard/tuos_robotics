# How to set up:

1. Place `fastdds.service` in `/etc/systemd/system/`
2. Run `sudo systemctl enable fastdds.service` 

## For troubleshooting purposes

- Use `sudo journalctl -u fastdds.service` to view logs
