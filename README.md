# NIOSR
Projekt zaliczeniowy przedmiot NIOSR, prowadzący zajęcia: mgr inż. Jakub Chudziński

Data demonstracji: 19 stycznia, godzina 13:30

## Odczyt danych IMU, węzeł usb_imu

W celu odczytu danych z IMU skorzystałem z płytki Arduino i sensora Groove IMU 9DOF. Na płytce została wgrana przykładowa [biblioteka](https://www.seeedstudio.com/Grove-IMU-9DOF-v2-0.html) utworzona przez producenta.
Dane są odczytywane przez port szeregowy, korzystając z biblioteki PySerial, w węźle: usb_imu.
Węzeł publikuje temat: 'imu/data_raw' dla danych IMU
węzeł publikuje temat: 'imu/mag' dla danych magnetometru 

## Węzeł camera_node

Węzeł jest rozbudowa zadania 3.go realizowanego na zajęciach pt: [Tworzenie i przetwarzanie węzłów](https://jug.dpieczynski.pl/lab-niodsr/Lab%2008%20-%20Tworzenie%20węzłów%20i%20przetwarzanie%20wiadomości.html). Po przechwyceniu obrazu pojawia się GUI, w którym można poprzez klikniecie utworzyć kwadratowy znacznik.
Węzeł został rozbudowany o detekcje obrazu ARuco, wykrycie generuje znacznik na wcześniej wspomnianym oknie. 

Węzeł publikuje: 'points', czyli koordynaty znacznika.

## Węzeł robot_control

Węzeł służy do publikowania wiadomosci do symulatora TURTLESIM, w celu sterowania żółwiem.

Subkrybuje nazwy tematu 'points', punkty powyżej środka połowy ekranu generują wiadomość ruchu do przodu, poniżej do tyłu.
Subkrybuje nazwy tematu 'imu/data', pierwsza wiadomość jest inizjalizacją orientacji, każda kolejna służy do wyliczenia zmiany orientacji pod kątem wyznaczenia obrotu w osi Z. Matematyka kwaternionów zostala zaimplementowana z wykorzystaniem biblioteki math, oraz dodano unittest w celu potwierdzenia działania.

## Filtr danych IMU

Surowe dane z IMU są przetwarzane przez Complementary_Filter_ROS z biblioteki [imu_tools](http://wiki.ros.org/imu_tools) dzięki czemu wiadomość jest uzupełniana o orientacje sensora.
Węzeł subskrybuje dane imu: 'imu/data_raw', 'imu/mag'
Publikuje:  'imu/data'
