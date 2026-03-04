# Nav2 Parametreleri – Detaylı Açıklama

Bu belge `nav2_params.yaml` dosyasındaki tüm parametreleri ve etkilerini açıklar.

---

## 1. bt_navigator (Behavior Tree Navigator)

| Parametre | Değer | Açıklama / Etkisi |
|-----------|--------|-------------------|
| **global_frame** | `map` | Tüm navigasyon hedefleri bu koordinat sisteminde tanımlanır. Yanlış frame → hedef yanlış yerde görünür. |
| **robot_base_frame** | `base_link` | Robotun referans noktası. TF ağacında bu frame'e göre pozisyon hesaplanır. |
| **odom_topic** | `/odom` | Odometri topic'i. Hız/pozisyon tahmini buradan alınır; yanlış topic → sürüklenme veya sapma. |
| **bt_loop_duration** | `10` (ms) | BT'nin bir döngüde ne kadar süre çalışacağı. Düşük = daha sık karar, yüksek = daha az CPU. |
| **default_server_timeout** | `20` (s) | Action server'lara varsayılan bekleme süresi. Bu sürede cevap gelmezse işlem iptal edilir. |
| **wait_for_service_timeout** | `1000` (ms) | Servislerin hazır olması için bekleme. Başlangıçta servisler geç gelirse artırılır. |
| **action_server_result_timeout** | `900.0` (s) | Navigasyon action'ının tamamlanması için max süre. Uzun rotalarda yüksek tutulur. |
| **navigators** | `navigate_to_pose`, `navigate_through_poses` | Hangi navigasyon modları açık. Tek hedef vs. waypoint listesi. |
| **error_code_names** | `compute_path_error_code`, `follow_path_error_code` | BT'de hata kodlarının isimleri; özelleştirilmiş recovery davranışları için kullanılır. |

---

## 2. controller_server (MPPI Controller)

### Genel

| Parametre | Değer | Açıklama / Etkisi |
|-----------|--------|-------------------|
| **controller_frequency** | `20.0` (Hz) | Kontrolcü çalışma frekansı. Yüksek = daha akıcı takip, daha fazla CPU. |
| **costmap_update_timeout** | `0.30` (s) | Costmap bu süreden eskiyse güncelleme beklenir; süre aşımında hata. |
| **min_x_velocity_threshold** | `0.001` | Bu hızın altında robot "duruyor" kabul edilir (ileri/geri). |
| **min_y_velocity_threshold** | `0.5` | Yanal hız eşiği. Tricycle'da yanal hareket yok; pratikte kullanılmaz. |
| **min_theta_velocity_threshold** | `0.001` | Bu açısal hızın altında dönüş "yok" sayılır. |
| **failure_tolerance** | `0.3` (m) | Hedefe bu mesafede kabul edilir; büyük = daha toleranslı, küçük = daha hassas. |
| **progress_checker_plugins** | `["progress_checker"]` | İlerleme kontrolü eklentisi; takılı kalınca recovery tetikler. |
| **goal_checker_plugins** | `["general_goal_checker"]` | Hedefe ulaşıldığını doğrulayan eklenti. |
| **controller_plugins** | `["FollowPath"]` | Kullanılan kontrolcü; burada sadece MPPI (path takip). |
| **use_realtime_priority** | `false` | `true` ise thread gerçek zamanlı öncelik alır; genelde simülasyonda false. |

### Progress checker

| Parametre | Değer | Açıklama / Etkisi |
|-----------|--------|-------------------|
| **required_movement_radius** | `0.5` (m) | Bu yarıçap içinde hareket yoksa "takılı" sayılır. |
| **movement_time_allowance** | `10.0` (s) | Bu süre içinde gerekli hareket olmazsa recovery (spin, backup vb.) tetiklenir. |

### Goal checker (general_goal_checker)

| Parametre | Değer | Açıklama / Etkisi |
|-----------|--------|-------------------|
| **stateful** | `True` | Bir kez hedefe "ulaştı" denirse tekrar kontrol etmez; titreşimi azaltır. |
| **xy_goal_tolerance** | `0.25` (m) | Hedef noktaya yatay mesafe toleransı. Küçük = hassas park, büyük = erken "tamamlandı". |
| **yaw_goal_tolerance** | `0.25` (rad) | Hedef yön toleransı. Küçük = tam yön hizalama, büyük = yönü gevşek bırakır. |

### FollowPath (MPPI Controller)

| Parametre | Değer | Açıklama / Etkisi |
|-----------|--------|-------------------|
| **time_steps** | `56` | Tahmin penceresindeki zaman adımı sayısı. Fazla = daha iyi tahmin, daha yavaş. |
| **model_dt** | `0.05` (s) | Her adımda 50 ms; toplam bakış süresi ≈ 56×0.05 ≈ 2.8 s. |
| **batch_size** | `2000` | Her adımda üretilen trajectory sayısı. Fazla = daha iyi seçenek, daha ağır hesaplama. |
| **ax_max / ax_min** | `3.0 / -3.0` | İleri/geri ivme limiti (m/s²). Fren/accelerasyon davranışını belirler. |
| **ay_max / ay_min** | `3.0 / -3.0` | Yanal ivme (tricycle'da kullanılmıyor). |
| **az_max** | `3.5` | Açısal ivme limiti (rad/s²). Ne kadar hızlı döneceği. |
| **vx_std** | `0.2` | İleri hız için örnekleme dağılımı. Büyük = daha çeşitli trajectory'ler. |
| **vy_std** | `0.0` | Tricycle'da yanal hareket yok; 0 mantıklı. |
| **wz_std** | `0.4` | Açısal hız dağılımı. Büyük = daha fazla dönüş seçeneği. |
| **vx_max / vx_min** | `0.5 / -0.35` | İleri/geri max hız. Geri daha düşük = geri gitmeyi sınırlar. |
| **vy_max** | `0.0` | Tricycle'da yanal hız yok. |
| **wz_max** | `1.9` (rad/s) | Maksimum dönüş hızı. |
| **iteration_count** | `1` | MPPI'da kaç iterasyon; 1 genelde yeterli, artırırsan kalite/hesaplama artar. |
| **prune_distance** | `1.7` (m) | Path'te robotun bu mesafe gerisindeki noktalar dikkate alınmaz; hesaplama azalır. |
| **transform_tolerance** | `0.1` (s) | TF'in ne kadar eski olabileceği; eski TF = hata. |
| **temperature** | `0.3` | MPPI'da seçim "yumuşaklığı". Düşük = en iyi trajectory'ye odaklanır. |
| **gamma** | `0.015` | Gelecek maliyetlerin indirgenme katsayısı. Düşük = kısa vadeli, yüksek = uzun vadeli plan. |
| **motion_model** | `"Ackermann"` | Tricycle/araba tipi; minimum dönüş yarıçapı (AckermannConstraints) ile uyumlu. |
| **visualize** | `true` | Trajectory'leri RViz'de gösterir; debug için faydalı. |
| **regenerate_noises** | `true` | Her adımda yeni rastgele örnekler; çeşitlilik sağlar. |
| **AckermannConstraints / min_turning_r** | `0.25` (m) | Minimum dönüş yarıçapı. Bu değerin altında dönüş trajectory'lere izin verilmez. |

### Critics (MPPI maliyetleri)

Her critic: **cost_weight** ne kadar yüksekse o kriter o kadar önemli; **cost_power** maliyetin doğrusal/süper doğrusal olmasını etkiler.

| Critic | Ağırlık | Ne yapar / Etkisi |
|--------|--------|--------------------|
| **ConstraintCritic** | 4.0 | Dinamik/geometrik kısıtları ihlal eden trajectory'leri cezalandırır. |
| **GoalCritic** | 5.0 | Hedefe uzak trajectory'leri cezalandırır. **threshold_to_consider: 1.4** m: bu mesafeden sonra hedef maliyeti devreye girer. |
| **GoalAngleCritic** | 3.0 | Hedef yönüne dönük olmayan trajectory'leri cezalandırır. **threshold_to_consider: 0.5** m. |
| **PreferForwardCritic** | 5.0 | Geri gitmeyi cezalandırır; **threshold_to_consider: 0.5** m'den sonra. |
| **CostCritic** | 2.8 | Costmap'teki yüksek maliyetli bölgelere giren trajectory'leri cezalandırır. **near_collision_cost: 253**, **critical_cost: 250**, **collision_cost: 1000000** ile engellere yaklaşma/çarpma çok pahalı. **consider_footprint: true** = robot boyutu dikkate alınır. **near_goal_distance: 1.0** = hedefe yakınken cost cezası hafifletilebilir. |
| **PathAlignCritic** | 14.0 | Path ile hizalanmayı teşvik eder. **max_path_occupancy_ratio: 0.08** = path biraz maliyetli olsa da kabul (sıkışmadan çıkmak için). **offset_from_furthest: 20**, **trajectory_point_step: 4** = path'in hangi kısmına bakılacağı. |
| **PathFollowCritic** | 5.0 | Path'i takip etmeyi teşvik eder; **offset_from_furthest: 5**. |
| **PathAngleCritic** | 2.0 | Path yönü ile robot yönünü hizalar; **max_angle_to_furthest**, **mode** ile davranış ayarlanır. |

---

## 3. local_costmap

| Parametre | Değer | Açıklama / Etkisi |
|-----------|--------|-------------------|
| **update_frequency** | `5.0` (Hz) | Ne sıklıkla güncellenir. Yüksek = güncel ortam, daha fazla CPU. |
| **publish_frequency** | `2.0` (Hz) | RViz'e ne sıklıkla yayınlanır. |
| **global_frame** | `odom` | Hareketli pencere odom'a göre; robotla birlikte "kayar". |
| **rolling_window** | `true` | Robot etrafında sabit boyutlu pencere; global map gerekmez. |
| **width / height** | `3` (m) | Pencere boyutu. Küçük = hızlı, büyük = daha fazla çevre bilgisi. |
| **resolution** | `0.05` (m/hücre) | 5 cm; küçük = detaylı, büyük = hızlı ve daha az bellek. |
| **footprint** | Polygon | Robot şekli (base_link'te). Çarpışma ve inflation bu şekle göre. |
| **inflation_layer / cost_scaling_factor** | `4.0` | Büyük = maliyet mesafeyle hızlı düşer, dar "mavi/mor" bant; robot sıkışmadan daha kolay çıkar. |
| **inflation_layer / inflation_radius** | `1.55` (m) | Engellerin etrafında bu yarıçapa kadar maliyet yayılır; footprint'in circumscribed radius'undan büyük olmalı. |
| **voxel_layer** | ... | 3D engelleri 2D costmap'e yansıtır. **z_voxels**, **max_obstacle_height**, **origin_z** yükseklik aralığını belirler. |
| **scan / raytrace_max_range** | `3.0` | Bu mesafeye kadar ışın atılarak engel arkası "temiz" işaretlenir. |
| **scan / obstacle_max_range** | `2.5` | Bu mesafenin ötesi engel olarak işaretlenmez (lidar güvenilirliği). |

---

## 4. global_costmap

Yapı local_costmap'e benzer; farklar:

| Parametre | Değer | Açıklama / Etkisi |
|-----------|--------|-------------------|
| **global_frame** | `map` | Tüm harita `map` frame'inde. |
| **update_frequency / publish_frequency** | `1.0` | Global olduğu için daha düşük frekans yeterli. |
| **track_unknown_space** | `true` | Bilinmeyen alanlar ayrı tutulur; planlayıcı bilinmeyene gidebilir (**allow_unknown** ile). |
| **plugins** | static + obstacle + inflation | Statik harita + canlı engeller + şişirme. |

---

## 5. planner_server

| Parametre | Değer | Açıklama / Etkisi |
|-----------|--------|-------------------|
| **expected_planner_frequency** | `20.0` (Hz) | Planlayıcının ne sıklıkla çalışması beklendiği. |
| **costmap_update_timeout** | `1.0` (s) | Costmap bu süreden eskiyse planlama bekler/hata verir. |
| **GridBased / tolerance** | `0.5` (m) | Hedef grid'e tam oturmasa da bu kadar yakınsa kabul. |
| **use_astar** | `false` | `true` = A*, `false` = Dijkstra. Dijkstra daha smooth path, A* daha hızlı. |
| **allow_unknown** | `true` | Bilinmeyen (keşfedilmemiş) alanlardan geçen path'e izin verir. |

---

## 6. smoother_server

| Parametre | Değer | Açıklama / Etkisi |
|-----------|--------|-------------------|
| **tolerance** | `1.0e-10` | Optimizasyon durma kriteri; küçük = daha hassas smooth. |
| **max_its** | `1000` | Maksimum iterasyon; yüksek = daha smooth, daha yavaş. |
| **do_refinement** | `True` | Ek ince ayar aşaması; path daha düzgün olur. |

---

## 7. behavior_server

Recovery ve yardımcı davranışlar (spin, backup, drive_on_heading, wait, assisted_teleop).

| Parametre | Değer | Açıklama / Etkisi |
|-----------|--------|-------------------|
| **cycle_frequency** | `10.0` (Hz) | Davranış ağacı döngü frekansı. |
| **simulate_ahead_time** | `2.0` (s) | Davranışların simüle edildiği süre (örn. çarpışma tahmini). |
| **max_rotational_vel** | `1.0` (rad/s) | Spin/rotate için max açısal hız. |
| **min_rotational_vel** | `0.4` | Min dönüş hızı. |
| **rotational_acc_lim** | `3.2` | Açısal ivme limiti. |

---

## 8. waypoint_follower

| Parametre | Değer | Açıklama / Etkisi |
|-----------|--------|-------------------|
| **loop_rate** | `20` (Hz) | Waypoint kontrol döngüsü. |
| **stop_on_failure** | `false` | Bir waypoint'te hata olunca durur mu yoksa devam mı. |
| **waypoint_pause_duration** | `200` (ms) | Her waypoint'te bekleme süresi. |

---

## 9. route_server

Graf tabanlı rota (opsiyonel).

| Parametre | Değer | Açıklama / Etkisi |
|-----------|--------|-------------------|
| **boundary_radius_to_achieve_node** | `1.0` (m) | Node'a bu mesafede "ulaşıldı" sayılır. |
| **radius_to_achieve_node** | `2.0` (m) | Alternatif ulaşma yarıçapı. |
| **smooth_corners** | `true` | Köşeleri yumuşatır. |
| **CollisionMonitor / max_collision_dist** | `3.0` (m) | Çarpışma kontrolü mesafesi. |

---

## 10. velocity_smoother

| Parametre | Değer | Açıklama / Etkisi |
|-----------|--------|-------------------|
| **smoothing_frequency** | `20.0` (Hz) | Çıkış komutlarının üretilme frekansı. |
| **max_velocity / min_velocity** | `[0.5, 0, 2.0]` vb. | [vx, vy, wz] limitleri; controller çıktısı bunu aşmaz. |
| **max_accel / max_decel** | `[2.5, 0, 3.2]` | İvme/fren limitleri; ani sarsıntı azalır. |
| **deadband_velocity** | `[0, 0, 0]` | Bu altındaki hızlar 0 yapılır; titreşim azalır. |
| **velocity_timeout** | `1.0` (s) | Bu süre komut gelmezse dur; güvenlik. |

---

## 11. collision_monitor

| Parametre | Değer | Açıklama / Etkisi |
|-----------|--------|-------------------|
| **source_timeout** | `1.0` (s) | Sensör verisi bu süre gelmezse güvenli mod. |
| **FootprintApproach / time_before_collision** | `1.2` (s) | Tahmini çarpışmaya bu kadar süre kala müdahale (yavaşlat/dur). |
| **simulation_time_step** | `0.1` (s) | Çarpışma simülasyonu zaman adımı. |
| **scan / min_height, max_height** | `0.15`, `2.0` (m) | Hangi yükseklik aralığındaki noktalar engel sayılır. |

---

## 12. docking_server

Şarj/dock'a yanaşma.

| Parametre | Değer | Açıklama / Etkisi |
|-----------|--------|-------------------|
| **controller_frequency** | `50.0` (Hz) | Dock kontrolcü frekansı. |
| **dock_approach_timeout** | `30.0` (s) | Yanaşma için max süre. |
| **undock_linear_tolerance** | `0.05` (m) | Dock'tan ayrılma konum toleransı. |
| **docking_threshold** | `0.05` (m) | Dock'a "ulaştı" sayılma mesafesi. |
| **staging_x_offset** | `-0.7` (m) | Dock önü bekleme noktası ofseti. |
| **controller / k_phi, k_delta** | `3.0`, `2.0` | Açı ve tekerlek açısı geri besleme kazançları. |
| **dock_collision_threshold** | `0.3` | Costmap'te bu maliyetin üzeri "çarpışma riski" sayılır. |

---

## 13. loopback_simulator

Simülasyon için döngü (odom ↔ cmd_vel).

| Parametre | Değer | Açıklama / Etkisi |
|-----------|--------|-------------------|
| **update_duration** | `0.02` (s) | 50 Hz güncelleme. |
| **scan_range_min/max** | `0.05`, `30.0` (m) | Simüle lidar menzili. |
| **scan_angle_min/max** | ±π | Tarama açısı. |
| **scan_angle_increment** | `0.02617` (rad) | Açı çözünürlüğü. |

---

## Özet – Hangi Parametre Neyi Etkiler?

- **Robot takılı kalıyorsa:** `progress_checker` (movement_time_allowance, required_movement_radius), `CostCritic` weight, `inflation_layer` (cost_scaling_factor, inflation_radius), `PathAlignCritic` (max_path_occupancy_ratio).
- **Hedefe tam ulaşmıyor / erken bitiyor:** `xy_goal_tolerance`, `yaw_goal_tolerance`, `failure_tolerance`, `GoalCritic` (threshold_to_consider).
- **Path'i kötü takip ediyor:** `PathFollowCritic`, `PathAlignCritic` ağırlıkları, MPPI `time_steps`, `batch_size`, `temperature`.
- **Çok yavaş / çok hızlı:** `vx_max`, `vx_min`, `wz_max`, velocity_smoother limitleri, `ax_max`/`ax_min`.
- **Dönüşler keskin / imkansız:** `AckermannConstraints / min_turning_r`, `wz_max`, tekerlek açısı limitleri (URDF ile uyumlu olmalı).
- **Engellere çok yaklaşıyor:** `CostCritic` (near_collision_cost, critical_cost), `inflation_radius`, `collision_monitor` (time_before_collision).
