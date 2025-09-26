# ROS2 Packages for publishing robot moves using keyboard

ใช้ webui ในการรับ keyboard event เนื่องจากถ้ารับจาก terminal พอกดค้าง ตอนแรกจะมี event กด และจะมีช่วงเว้นว่าง (no event) แล้ว terminal ถึงจะส่ง event กดค้างให้
ถ้าใช้ webui จะรับ keyboard event ได้ตรงตามความจริง

## ทำไมไม่ทำ gui? (tkinter, gtk, qt..)
เพราะ webui มัน cross platform และทะลุ docker ได้
