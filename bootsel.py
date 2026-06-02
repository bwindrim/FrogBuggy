import sys, time, rp2
print(f"{sys.implementation._machine=}\n  {sys.version=}\n")
print(f"press bootsel_button")
for i in range(5, 0, -1):
    print(f"{i:2} {rp2.bootsel_button()=}")
       # result @ Pico as expected: 1 if button pressed, else 0
       # result @ Pico2: always 1
    time.sleep(1)