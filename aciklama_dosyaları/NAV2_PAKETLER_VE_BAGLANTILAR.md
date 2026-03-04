# Nav2 Paketleri: Bağlantılar ve Görevler

Bu belge Nav2’deki ana bileşenleri, birbirleriyle nasıl konuştuğunu ve ne iş yaptıklarını özetler.

---

## 1. Genel Akış (Kim Kime Bağlı?)

```
[Kullanıcı / Uygulama]
         │
         │ hedef (pose) veya waypoint listesi
         ▼
┌─────────────────────────────────────────────────────────────────┐
│  bt_navigator (Behavior Tree Navigator)                          │
│  "Orkestra şefi": Hangi sırayla hangi server çağrılacak?"        │
└─────────────────────────────────────────────────────────────────┘
         │
         │ Behavior Tree ile sırayla çağırır:
         │
         ├──► planner_server      (path üret)   ──► global_costmap kullanır
         │
         ├──► controller_server   (path takip)  ──► local_costmap kullanır
         │         │
         │         └──► cmd_vel (ham) ──► velocity_smoother ──► cmd_vel_smoothed
         │                                      │
         │                                      └──► collision_monitor ──► cmd_vel (motora)
         │
         ├──► behavior_server     (recovery: spin, backup, wait, ...)
         │         └──► local_costmap, global_costmap (footprint vb.)
         │
         └──► (replan / recovery gerekirse tekrar planner veya behavior)
```

- **Path’i çizen:** planner_server (global costmap ile).  
- **Path’i takip ettiren:** controller_server (local costmap ile).  
- **Hepsini sıraya sokan:** bt_navigator (Behavior Tree).

---

## 2. Bileşenler Tek Tek

### 2.1 bt_navigator (nav2_bt_navigator)

| Ne işe yarar? | Kullanıcı hedefi alır; hangi adımın ne zaman yapılacağına Behavior Tree ile karar verir. |
|---------------|-------------------------------------------------------------------------------------------|
| **Girdi**     | Action: `navigate_to_pose` veya `navigate_through_poses` (hedef/waypoint’ler).          |
| **Çıktı**     | Sırayla planner_server, controller_server, behavior_server’ı çağırır (service/action).    |
| **Bağlantı** | Planner’dan path alır → Controller’a “bu path’i takip et” der → Hedefe ulaşınca veya hata olunca recovery (behavior_server) tetikler. |

Özet: Path çizme ve path takip işini yapan **planner** ve **controller** değil; **bt_navigator** onları sırayla çalıştırır ve gerektiğinde replan/recovery yapar.

---

### 2.2 planner_server (nav2_planner)

| Ne işe yarar? | Başlangıç → hedef arasında, engellere çarpmayan bir **global path** (yeşil çizgi) üretir. |
|---------------|--------------------------------------------------------------------------------------------|
| **Girdi**     | ComputePathToPose service (start, goal); **global_costmap**.                             |
| **Çıktı**     | Path (pose dizisi).                                                                       |
| **Bağlantı**  | **global_costmap**’i okur (static + obstacle + inflation). Path’i **smoother_server**’a gönderir (opsiyonel). |

Özet: **Path’i çizen** bileşen budur; path **global costmap** üzerinde hesaplanır.

---

### 2.3 smoother_server (nav2_smoother)

| Ne işe yarar? | Planner’ın ürettiği path’i yumuşatır (keskin köşeleri azaltır).                          |
|---------------|-------------------------------------------------------------------------------------------|
| **Girdi**     | SmoothPath service ile path.                                                             |
| **Çıktı**     | Yumuşatılmış path.                                                                       |
| **Bağlantı**  | bt_navigator, planner’dan path aldıktan sonra (isteğe bağlı) smoother’ı çağırır; sonuç controller’a gider. |

Özet: Path’i **çizmez**, sadece **düzenler**; doğrudan costmap kullanmaz.

---

### 2.4 controller_server (nav2_controller + nav2_mppi_controller)

| Ne işe yarar? | Verilen path’i **takip ettirir**; engellere çarpmadan **cmd_vel** (hız komutu) üretir.   |
|---------------|-------------------------------------------------------------------------------------------|
| **Girdi**     | Path (planner/smoother’dan); **local_costmap**; robot pozisyonu (TF / odom).              |
| **Çıktı**     | `cmd_vel` (veya senin kurulumda sonra velocity_smoother’a giden topic).                   |
| **Bağlantı**  | **local_costmap**’i kullanır (obstacle + inflation). Progress checker “takıldı mı?”, goal checker “hedefe vardık mı?” bakar. |

Özet: **Path’i takip ettiren** ve **local costmap’i kullanan** bileşen controller’dır; path’i çizmez.

---

### 2.5 local_costmap & global_costmap (nav2_costmap_2d)

| Costmap        | Frame / Pencere     | Kim kullanır?              | Ne için?                                      |
|----------------|--------------------|----------------------------|-----------------------------------------------|
| **global_costmap** | `map`, tüm harita  | planner_server, behavior_server | Path planlama; recovery davranışları.          |
| **local_costmap**  | `odom`, rolling window | controller_server, behavior_server | Path takip; anlık engellerden kaçınma.        |

- **Katmanlar (senin config):**  
  - Global: static_layer + obstacle_layer + inflation_layer.  
  - Local: obstacle_layer + inflation_layer (static yok, odom kayması olmasın diye).

Özet: **Path’i global costmap üzerinde planlar;** **local costmap path’i çizmez**, sadece **takip ve kaçınma** için kullanılır.

---

### 2.6 behavior_server (nav2_behaviors)

| Ne işe yarar? | Recovery davranışları: Takılınca veya hatada **spin**, **backup**, **wait**, **drive_on_heading** vb. |
|---------------|------------------------------------------------------------------------------------------------------|
| **Girdi**     | Behavior Tree’den çağrı; **local_costmap**, **global_costmap** (footprint topic’leri vb.).           |
| **Çıktı**     | Geçici süre `cmd_vel` (dön, geri git, bekle, …).                                                    |
| **Bağlantı**  | bt_navigator hata/ilerleme yok dediğinde behavior_server’ı çalıştırır; costmap’lere bakar.          |

Özet: Path çizmez, path takip etmez; **sadece “takıldık / hata” durumunda** kurtarma davranışı yapar.

---

### 2.7 velocity_smoother (nav2_velocity_smoother)

| Ne işe yarar? | Controller’dan gelen hız komutunu **ivme/sarsıntı sınırlarıyla** yumuşatır.              |
|---------------|-------------------------------------------------------------------------------------------|
| **Girdi**     | `cmd_vel` (controller çıktısı).                                                          |
| **Çıktı**     | `cmd_vel_smoothed` (veya senin kurulumda collision_monitor’a giden topic).                |
| **Bağlantı**  | Controller → velocity_smoother → (opsiyonel) collision_monitor → nihai `cmd_vel`.        |

Özet: Path veya costmap ile ilgisi yok; sadece **hız profilini** düzenler.

---

### 2.8 collision_monitor (nav2_collision_monitor)

| Ne işe yarar? | Son anda çarpışma riski varsa hızı **düşürür veya durdurur** (footprint + scan).         |
|---------------|-------------------------------------------------------------------------------------------|
| **Girdi**     | `cmd_vel_smoothed`; `/scan`; footprint (local_costmap’ten).                              |
| **Çıktı**     | Güvenli `cmd_vel` (motora giden).                                                         |
| **Bağlantı**  | velocity_smoother’dan alır; costmap’i doğrudan path için kullanmaz, sadece güvenlik.       |

Özet: Path çizmez; **son kademe güvenlik**; path’i takip ettiren yine controller’dır.

---

### 2.9 waypoint_follower (nav2_waypoint_follower)

| Ne işe yarar? | Waypoint listesini sırayla hedef gibi **navigate_through_poses** ile gönderir; isteğe göre her waypoint’te bekler. |
|---------------|---------------------------------------------------------------------------------------------------------------------|
| **Girdi**     | Waypoint listesi (pose’lar).                                                                                       |
| **Çıktı**     | bt_navigator’ın `navigate_through_poses` action’ına hedefler gönderir.                                             |
| **Bağlantı**  | Path’i çizen yine planner (global costmap); waypoint_follower sadece “sıradaki hedefi kim nereye gönderecek”ı yönetir. |

Özet: Path çizmez; **çok noktalı görevleri** bt_navigator’a bağlar.

---

### 2.10 route_server (nav2_route) [Opsiyonel]

| Ne işe yarar? | Graf tabanlı rota (lane/waypoint grafı), hız limiti, rerouting, collision monitor.       |
|---------------|-------------------------------------------------------------------------------------------|
| **Bağlantı**  | Özel kullanım; standart “path çiz / takip et” zincirinin dışında.                       |

---

### 2.11 docking_server (nav2_docking / opennav_docking) [Opsiyonel]

| Ne işe yarar? | Şarj istasyonu / dock’a yanaşma ve ayrılma.                                             |
|---------------|-------------------------------------------------------------------------------------------|
| **Bağlantı**  | Local costmap, footprint; path planlama değil, özel docking kontrolü.                   |

---

## 3. Topic / Service Özeti (Bağlantılar)

| Kaynak              | Hedef / Topic-Service          | Ne için?                    |
|---------------------|--------------------------------|-----------------------------|
| Kullanıcı            | bt_navigator action            | Hedef / waypoint listesi    |
| bt_navigator        | planner_server (service)      | Path isteği                 |
| planner_server      | global_costmap (okur)         | Path planlama               |
| planner_server      | smoother_server (opsiyonel)   | Path yumuşatma              |
| bt_navigator        | controller_server (action)    | Path takip isteği           |
| controller_server   | local_costmap (okur)          | Takip + engelden kaçınma    |
| controller_server   | cmd_vel → velocity_smoother   | Hız komutu                  |
| velocity_smoother   | cmd_vel_smoothed              | Yumuşatılmış hız            |
| collision_monitor   | cmd_vel_smoothed → cmd_vel    | Son çarpışma kontrolü       |
| bt_navigator        | behavior_server               | Recovery (spin, backup, …)  |
| behavior_server     | local/global costmap (okur)   | Recovery planlama           |
| /map                | global_costmap (static_layer) | Statik harita               |
| /scan               | local/global costmap (obstacle_layer) | Canlı engeller      |
| /odom               | bt_navigator, controller      | Robot pozisyonu / hız       |

---

## 4. Kısa Özet

- **Path’i çizen:** **planner_server** → **global_costmap** kullanır.  
- **Path’i takip ettiren:** **controller_server** → **local_costmap** kullanır.  
- **Hangi adımın ne zaman yapılacağı:** **bt_navigator** (Behavior Tree).  
- **Recovery:** **behavior_server** (costmap’leri okur, path çizmez).  
- **Hız yumuşatma:** **velocity_smoother**; **son güvenlik:** **collision_monitor**.

Bu belge, `nav2_params.yaml` ve senin forklift kurulumundaki yapıya göre yazıldı; route_server ve docking_server opsiyonel kullanım içindir.
