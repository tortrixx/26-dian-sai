import network, time

print("====== WiFi 连接测试 ======")
wlan = network.WLAN(network.STA_IF)
print("active:", wlan.active())

SSID = "test"
PASS = "90z5M92#"

print("connect to:", SSID)
wlan.connect(SSID, PASS)

for i in range(15):
    time.sleep_ms(1000)
    ok = wlan.isconnected()
    print("[{}s] connected={}  ip={}".format(i+1, ok, wlan.ifconfig()[0]))
    if ok:
        print("====== WiFi OK! IP:", wlan.ifconfig()[0], "======")
        break
else:
    print("====== WiFi FAIL ======")
