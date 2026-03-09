# Offset Tricycle Robot

Gazebo Sim ile forklift robot simülasyonu ve Nav2 navigasyon paketi.

## Kurulum (Başka Bilgisayarda)

Paketi derleyip workspace'i source edin:

```bash
cd /path/to/forklift
colcon build --packages-select offset_tricycle_robot
source install/setup.bash
```

**Önemli:** `config/amcl.yaml` dosyasında **satır 53**'teki `yaml_filename` değerini kendi harita yolunuza göre güncelleyin. Bu yol mutlaka sizin bilgisayarınızdaki `warehouse.yaml` dosyasının tam yolunu göstermelidir.

```yaml
# config/amcl.yaml satır 52-53
map_server:
  ros__parameters:
    yaml_filename: "/SIZIN/YOLUNUZ/forklift/src/offset_tricycle_robot/maps/map/warehouse.yaml"
```

## Çalıştırma Sırası

Aşağıdaki komutları **ayrı terminallerde** sırayla çalıştırın.

**Not:** Başka bir bilgisayarda çalıştırıyorsanız, `params_file:=` ile verilen yolları kendi forklift workspace yolunuza göre değiştirin (örn. `/home/melih/Desktop/forklift` → `/sizin/yolunuz/forklift`).

```bash
# 1. Simülasyon
ros2 launch offset_tricycle_robot offset_tricycle_robot_sim.launch.py

# 2. Lokalizasyon (AMCL)
ros2 launch nav2_bringup localization_launch.py use_sim_time:=true params_file:=/home/melih/Desktop/forklift/src/offset_tricycle_robot/config/amcl.yaml

# 3. Nav2 bringup (localization ve slam kapalı)
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True use_localization:=False slam:=False params_file:=/home/melih/Desktop/forklift/src/offset_tricycle_robot/config/nav2_params.yaml

# 4. Keepout filter
ros2 launch offset_tricycle_robot keepout_filter.launch.py
```
