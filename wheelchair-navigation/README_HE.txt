סימולציית Webots מותאמת לגישה החדשה: C API + Controller.lib בלבד

מה הסימולציה כוללת:
- עולם: worlds/human_wheelchair_live.wbt
- כיסא גלגלים עם controller בשם: wheelchair_cpp_controller
- מנועים תואמים לקוד:
  left_wheel_motor
  right_wheel_motor
- Encoders:
  left_wheel_encoder
  right_wheel_encoder
- חיישנים תואמים לקוד:
  gps
  imu
  accelerometer
  gyro
  front_lidar
  left_lidar
  right_lidar
  rgb_camera
  front_radar_device

שיטת עבודה מומלצת:
1. חלצי את תוכן התיקייה TrackObject_webots_c_api_sync לתוך:
   C:\Users\User\Desktop\TrackObject

   כך שיהיו לך בתוך הפרויקט:
   C:\Users\User\Desktop\TrackObject\worlds\human_wheelchair_live.wbt
   C:\Users\User\Desktop\TrackObject\controllers\wheelchair_cpp_controller\

2. ודאי שה-build הצליח ושהקובץ קיים:
   C:\Users\User\Desktop\TrackObject\controllers\wheelchair_cpp_controller\wheelchair_cpp_controller.exe

3. פתחי ב-Webots את:
   worlds\human_wheelchair_live.wbt

4. לחצי Play.

אם חילצת את הסימולציה לתיקייה נפרדת, הריצי:
copy_built_controller_here.bat

הערה:
הכיסא עצמו אינו משתמש ב-Python. Python נשאר רק עבור הולכי רגל ורמזור.
