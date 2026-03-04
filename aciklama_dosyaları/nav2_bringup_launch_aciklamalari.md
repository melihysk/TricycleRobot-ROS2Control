# Nav2 Bringup Launch Dosyaları (`/opt/ros/jazzy/share/nav2_bringup/launch/`)

## 1. `bringup_launch.py`
**Amaç:** Tüm Nav2 stack'ini tek seferde başlatır (tek giriş noktası).
- **slam=False, use_localization=True** → `localization_launch.py` çalışır (map_server + AMCL, hazır harita).
- **slam=True, use_localization=True** → `slam_launch.py` çalışır (SLAM ile harita oluşturma).
- Her durumda **navigation_launch.py** çalışır (controller, planner, costmap, BT navigator vb.).
- Parametreler tek bir `params_file` (YAML) ile verilir.

---

## 2. `localization_launch.py`
**Amaç:** Sadece harita yükleme + lokalizasyon (AMCL).
- Başlattığı node'lar: **map_server**, **amcl**, **lifecycle_manager** (sadece bu ikisi için).
- Hazır harita kullanırken bunu kullanırsınız; sonrasında ayrıca navigation/bringup başlatılabilir.

---

## 3. `navigation_launch.py`
**Amaç:** Lokalizasyon **olmadan** sadece navigasyon bileşenleri (harita + AMCL zaten çalışıyorsa bunu eklersiniz).
- Başlattığı node'lar: **controller_server**, **smoother_server**, **planner_server**, **route_server**, **behavior_server**, **velocity_smoother**, **collision_monitor**, **bt_navigator**, **waypoint_follower**, **docking_server**, **lifecycle_manager_navigation**.
- Costmap parametreleri YAML’dan okunur; costmap, controller ve planner içinde kullanılır.

---

## 4. `slam_launch.py`
**Amaç:** SLAM (slam_toolbox) + map_saver ile harita oluşturma.
- slam_toolbox’ı başlatır; `/map` ve map→odom TF üretir.
- Harita kaydetmek için map_saver lifecycle’ı kullanılır.

---

## 5. `rviz_launch.py`
**Amaç:** Nav2 için hazır RViz konfigürasyonunu başlatır (harita, costmap, goal pose, robot model vb.).

---

## 6. `tb3_simulation_launch.py` / `tb4_simulation_launch.py`
**Amaç:** TurtleBot 3 / TurtleBot 4 simülasyonu + Nav2 (Gazebo + bringup birlikte). Kendi robotunuz için kullanmazsınız.

---

## 7. `tb3_loopback_simulation.launch.py` / `tb4_loopback_simulation.launch.py`
**Amaç:** Gerçek robot olmadan Nav2’yi test etmek için loopback simülatörü (sanal robot + sanal lidar). Kendi robotunuz için gerekmez.

---

## 8. `cloned_multi_tb3_simulation_launch.py` / `unique_multi_tb3_simulation_launch.py`
**Amaç:** Birden fazla TurtleBot 3 ile simülasyon ve Nav2 (multi-robot). Tek robot için gerekmez.

---

## Sizin akış (hazır harita + sonra navigasyon)

1. **Lokalizasyon:** `localization_launch.py` (map_server + AMCL) → harita yüklenir, AMCL çalışır.
2. **Navigasyon (bringup):** `bringup_launch.py` kullanırsanız o zaten hem localization hem navigation’ı açar; params_file’da hem map_server/AMCL hem de navigation parametreleri olmalı.
   - Alternatif: Önce localization_launch, ardından ayrı bir terminalde **navigation_launch.py** (sadece navigasyon) de açılabilir; her iki durumda da **bringup için gerekli tüm parametreler YAML’da açık olmalı**.
