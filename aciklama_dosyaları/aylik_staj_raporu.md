# STAJ ARA RAPORU - KAPAK SAYFASI

**Proje/Staj Adı:** ROS2 Tabanlı Özelleştirilmiş Tricycle Mobil Robot Geliştirme ve Navigasyonu
**Stajyer:** [Adınız Soyadınız]
**Staj Yapılan Kurum:** Modoya
**İlgili Dönem:** [Başlangıç Tarihi] - [Bitiş Tarihi] (1 Aylık Süreç)

---

*(Not: Sayfa sonu - Raporun İçindekiler kısmı yeni sayfadan başlamalıdır)*

<div style="page-break-after: always;"></div>

# İÇİNDEKİLER

1. Giriş
2. Şirket Hakkında: Modoya
3. Birinci Hafta: Araç Kinematiği, Tricycle Modeli ve ROS2 Ortam Kurulumu
   3.1. Modoya ve Otonom Araç Vizyonu
   3.2. Tricycle Araç Kinematiği ve Özelleştirilmiş Model (Offset)
   3.3. ROS2 Geliştirme Ortamının Hazırlanması
4. İkinci Hafta: Simülasyon ve Kontrolcü (Controller) İmplementasyonu
   4.1. Simülasyon Ortamında Robot Modelinin Çıkarılması
   4.2. Tricycle Kontrolcüsünün Offsetli Yapıya Uyarlanması
   4.3. Odometri Testleri ve Gözlemler
5. Üçüncü Hafta: Sensör Füzyonu, Lidar Odometrisi ve Dockerizasyon
   5.1. Genişletilmiş Kalman Filtresi (EKF) ile Odometri Füzyonu
   5.2. Lidar Odometrisi Paketlerinin Karşılaştırılması (rf2o ve scan_matcher)
   5.3. Proje Dağıtımı İçin Docker Entegrasyonu
6. Dördüncü Hafta: SLAM, Navigasyon (Nav2) ve Konumlandırma (AMCL)
   6.1. SLAM Toolbox ile Harita Üretimi
   6.2. Otonom Seyrüsefer İçin Nav2 Entegrasyonu
   6.3. AMCL ile Harita Üzerinde Konumlandırma
7. Mesleki, Teknik ve İş Hayatı Kazanımları
   7.1. Teknik ve Pratik Kazanımlar
   7.2. İş Çevresi, Süreç Yönetimi ve Takım Çalışması
8. Sonuç

---

<div style="page-break-after: always;"></div>

# 1. GİRİŞ

Bu rapor, Modoya bünyesinde gerçekleştirilen uzun dönemli staj çalışmasının ilk bir aylık sürecini kapsamaktadır. Otonom mobil robotların hem endüstriyel ortamlarda hem de iç lojistik (intralojistik) çözümlerinde artan önemi, bu cihazların hareket kabiliyetlerini, kinematik modellerini ve algı yeteneklerini optimize etmeyi zaruri kılmaktadır. Staj süresince, temel bir "Tricycle" (üç tekerlekli) araç kinematiğine sahip bir mobil robotun sıfırdan oluşturulması, üzerine özelleştirilmiş bir offset (kayıklık) eklenerek kontrolcü (controller) yapılarının düzenlenmesi ve nihayetinde navigasyon/SLAM algoritmalarının koşuşturulması hedeflenmiştir.

Çalışma kapsamında Robot İşletim Sistemi (ROS2) kullanılmış olup, tüm süreç modüler, ölçeklenebilir ve aktarılabilir bir yaklaşımla yürütülmüştür. Rapor boyunca her hafta tamamlanan görevler ardışık bir düzenle sunulacak; araştırma, tasarım, uygulama ve test aşamalarının akademik ve mekanik çıktıları detaylandırılacaktır.

# 2. ŞİRKET HAKKINDA: MODOYA

Staj faaliyetlerinin yürütüldüğü Modoya (https://www.modoya.com/), endüstriyel otomasyon ve özel otonom araç çözümleri üzerine Ar-Ge ve üretim faaliyetleri gösteren, teknoloji odaklı bir şirkettir. Otonom taşıma sistemleri (AMR/AGV), otonom forkliftler ve depolama araçları şirketin temel uzmanlık alanlarındandır. Şirket bünyesindeki mühendislik süreçleri, sadece yazılım değil, aynı zamanda kompleks donanımların ve mekanik sistemlerin gerçek zamanlı kontrolüne dayanmaktadır. Şirketin projelerinde yüksek hassasiyetli seyrüsefer algoritmaları ve çeşitli sensör (Lidar, IMU, derinlik kamerası) mimarileri yoğun olarak kullanılmaktadır. Modoya bünyesinde çalışmak, bir mobil robotun tasarımından sahada otonom olarak devriye gezecek seviyeye gelene kadarki tüm yaşam döngüsünü deneyimlemek için eşsiz bir zemin sunmuştur.

# 3. BİRİNCİ HAFTA: ARAÇ KİNEMATİĞİ, TRICYCLE MODELİ VE ROS2 ORTAM KURULUMU

İlk hafta çalışmaları, projenin temellerinin atılması, mevcut donanım senaryosunun anlaşılması ve yazılım-geliştirme altyapısının standardize edilmesi etrafında şekillenmiştir. 

### 3.1. Araç Kinematikleri ve Tricycle Modeli

Farklı mobil robot kinematikleri (diferansiyel sürüş, Ackermann sürüşü, omni-yönlü sürüş vb.) üzerine detaylı literatür taraması gerçekleştirilmiştir. Projenin ana odağı olan Tricycle (Üç tekerlekli) araç tipi detaylı bir biçimde incelenmiştir. Klasik tricycle kinematiğinde önde (veya arkada) yönlendirilebilir ve aynı zamanda tahrik verebilen bir tekerlek (steer wheel) ile, arkada sadece serbestçe dönen iki adet pasif tekerlek bulunmaktadır. Bu model, dar alanlarda yüksek manevra kabiliyeti sağlamakla birlikte, diferansiyel sistemlere kıyasla daha kompleks bir matematiksel model gerektirmektedir.

### 3.2. Özelleştirilmiş Offsetli Tricycle Kinematiği Üzerine İncelemeler

Firmada geliştirilen mevcut robotun, standart bir tricycle kinematiğinden farklı olarak yapısal bir "offset" (şekilsel veya eksenel kayıklık) barındırdığı tespit edilmiştir. Yönlendirme tekerleğinin tam simetri ekseninde yer almaması veya dönüş ekseninin tekerlek merkezinden farklı olması gibi geometrik sapmalar, dönüş (steering) açılarının ve tekerlek hızlarının standart formüllerle hesaplanmasını geçersiz kılmaktadır. Bu bağlamda, offsetli modelin kinematik denklemleri (İleri Kinematik ve Ters Kinematik) taslak olarak analiz edilmiş ve simülasyon ortamında gerçeklenmek üzere matematiksel olarak yeniden formüle edilmiştir.

### 3.3. ROS2 Geliştirme Ortamının Hazırlanması

Tüm bu algoritmik çözümleri test edebilmek amacıyla işletim sistemi üzerinde süreçlere başlanmıştır. Açık kaynak kodlu ve modern iletişim standartlarına (DDS - Data Distribution Service) sahip olan ROS2 (Robot Operating System 2) kurulumları tamamlanmıştır. Çalışma ortamının stabil çalışmasını teyit etmek için test paketleri (örneğin turtlesim ve rqt mekanizmaları) başlatılmış, CMake ve package.xml yapıları arasındaki bağımlılık zincirleri incelenmiştir. 

# 4. İKİNCİ HAFTA: SİMÜLASYON VE KONTROLCÜ (CONTROLLER) İMPLEMENTASYONU

İkinci hafta süresince, teorik olarak kurgulanan aracın sanal ortamda doğrulanması adına simülasyon ve kod düzeyinde robot-kontrol entegrasyonlarına ağırlık verilmiştir.

### 4.1. Simülasyon Ortamında Robot Modelinin Çıkarılması

Sanal ortam doğrulama süreçlerini başlatmak için hedef aracın CAD bilgilerinden yararlanılarak, baştan sona XACRO/URDF formatında (Unified Robot Description Format) robot tanımlamaları gerçekleştirilmiştir. Bu tanımlamalar içerisine offsetli yapıyı ifade edecek şekilde "link" ve "joint" dönüşümleri eklenmiştir. Robotun simülasyonda yerçekimi ve sürtünme gibi temel fiziksel etkilere doğru tepki verebilmesi adına atalet matrisleri (inertia matrix) hesaplanarak XML yapısına gömülmüştür. Yapılan tasarım Şekil 1'de ayrıntılı bir biçimde gösterilmiştir (Bkz. Şekil 1, Simülasyondaki Robot Modeli Görseli).

### 4.2. Tricycle Kontrolcüsünün Offsetli Yapıya Uyarlanması

Sanal ortamda robotun sürülebilmesi için `ros2_control` iskeleti altında bir kontrolcü implemente edilmiştir. ROS2 kütüphanesindeki standart `tricycle_controller` modülü kaynak kod düzeyinde incelenerek, birinci hafta belirlenen ve offset yapısından kaynaklı yeni kinematik formüller ilgili sınıflara (class) entegre edilmiştir. 

```cpp
// Örnek: Offsetli Yapı İçin Hedef Açı ve Hız Hesaplama Fonksiyonu İçinden Bir Kesit
// Standart modelde tekerlek merkezdeyken, offset hesaba katılarak alfa açısı ve tekerlek hızı (v_wheel) güncellenir.
double offset_y = 0.15; // Aracın yanal eksendeki tekerlek sapması
double adjusted_wheel_speed = calculate_wheel_speed(linear_x, angular_z, offset_y);
double adjusted_steering_angle = calculate_steering_angle(linear_x, angular_z, offset_y);

hardware_interface_velocity_command = adjusted_wheel_speed;
hardware_interface_steering_command = adjusted_steering_angle;
```

Yukarıdaki kod kesitinde ifade edildiği üzere, sistemden gelen lineer ve açısal (cmd_vel) hız referansları, dinamik offset değeri üzerinden kompanze edilerek donanım arayüzüne (hardware interface) basılmıştır. Modelde, offsetli yapının yarattığı asimetrinin kontrolcü seviyesinde de stabilite sorunları yaşatmaması hedeflenmiştir.

### 4.3. Odometri Testleri ve Gözlemler

Motor hızlarından ve dönüş açılarından geriye doğru yapılan hesaplama ile aracın bulunduğu konumu kestirmesine "Tekerlek Odometrisi" denmektedir. Geliştirilen yeni offsetli kontrolcü üzerinden tekerlek sensörleri (encoder verileri) okunarak bir odometri (odom -> base_link) tf yapısı üretilmiş ve robot simülasyonda farklı rotalarda sürüldüğünde sapmaların ne ölçüde olduğu RViz2 üzerinden detaylıca incelenmiştir.

# 5. ÜÇÜNCÜ HAFTA: SENSÖR FÜZYONU, LİDAR ODOMETRİSİ VE DOCKERİZASYON

Saf tekerlek odometrisinin zamanla kayma eğilimi (drift) göstermesi nedeniyle, sistemin daha istikrarlı bir konum tespitine sahip olabilmesi adına sensör füzyonu yaklaşımına geçiş yapılmıştır.

### 5.1. Genişletilmiş Kalman Filtresi (EKF) ile Odometri Füzyonu

Robottaki hatayı daha verimli bir şekilde asgari seviyeye indirmek için `robot_localization` paketinde bulunan EKF (Extended Kalman Filter) düğümleri kullanılmıştır. Tekerleklerden gelen nispeten stabil ancak kümülatif olarak kayan odometri verisi ile, sensörlerden (örneğin IMU/Lidar) gelen yönelim verileri tek bir matris yapısında birleştirilmiş olup, ortamdaki gürültülere karşı filtrelenmiş daha kararlı bir odometri çerçevesi yayını başlatılmıştır.

### 5.2. Lidar Odometrisi Paketlerinin Karşılaştırılması

EKF uygulamasının pekiştirilmesi ve kaymayı minimuma indirebilmek adına, lazer taranımındaki değişimler üzerinden tahmini hız vektörü çıkaran "Lidar Odometrisi" yaklaşımları araştırılmıştır. 
İlk olarak `rf2o_laser_odometry` paketi kurularak, 2D Lidar verisi üzerinden taramaların (scan matching) ardışık hareket tespiti yapılmıştır. Ardından alternatif bir yaklaşım olarak `ros2_laser_scan_matcher` paketi entegre edilmiş ve iki metodun sapma oranları karşılaştırılmıştır. Bu karşılaştırmadaki veri analizleri tablo haline getirilmiş ve işlemci performanslarına etkileri raporlanmıştır (Bkz. Tablo 1, Lidar Odometri Paketlerinin İşlemci Yükü ve Hassasiyet Karşılaştırması). Elde edilen veriler doğrultusunda projeye uygun olan yöntem aktif olarak seçilmiştir.

### 5.3. Proje Dağıtımı İçin Docker Entegrasyonu

Geliştirilen otonom yazılım bileşenlerinin farklı cihazlarda (örneğin doğrudan sahada çalışan test bilgisayarında veya bir AMR'nin endüstriyel pc ünitelerinde) herhangi bir konfigürasyon problemi çıkarılmadan çalıştırılabilmesi için proje Docker konteyneri (containerization) haline getirilmiştir. Hazırlanan Dockerfile içerisine gerekli tüm ros2 bağımlılıkları girilmiş olup, "yalnızca derlendiği sistemde çalışma" sorunu efektif bir şekilde giderilmiştir.

# 6. DÖRDÜNCÜ HAFTA: SLAM, NAVİGASYON (NAV2) VE KONUMLANDIRMA (AMCL)

Dördüncü haftada odak noktası, hareketi doğrulanan mobil robotun çevre ile olan etkileşimi, dinamik çevresini algılaması ve hedefe insansız bir şekilde gidebilmesi algoritmalarına kaymıştır.

### 6.1. SLAM Toolbox ile Harita Üretimi

Elde edilen kararlı odometri yapısı, çevresel 2D Lidar lazer nokta bulutu ile beslenerek robotta Eşzamanlı Konumlandırma ve Haritalama (SLAM) sürecine geçilmiştir. Bu kapsamda ROS2 mimarisinin güncel paketlerinden olan `slam_toolbox` entegre edilmiştir. Simülasyon vasıtasıyla oluşturulmuş rastgele test alanlarında, robota manuel sürüş komutları verilerek kapalı alanların hücresel tarama haritası (Occupancy Grid Map - OGM) başarıyla çıkartılmış ve kaydedilmiştir. Oluşturulan Grid Map, çalışmadaki topolojinin anlaşılması açısından görselleştirilmiştir (Bkz. Şekil 2, Çıkarılan 2B Çevre Haritası).

### 6.2. Otonom Seyrüsefer İçin Nav2 Entegrasyonu

Haritası çıkarılan alan üzerinde planlı rota algoritmaları (path planning) için ROS2 Navigation2 (Nav2) yığını çalışmaya dahil edilmiştir. Küresel ve yerel maliyet haritaları (Global and Local Costmaps) konfigüre edilerek robotun statik duvarlara ve dinamik engellere çarpmasını engelleyecek bir tolerans ve emniyet katmanı (inflation radius vb.) kurgulanmıştır. Tricycle modelinin kinematik özelliklerine bağlı dönme çapı gibi sınırlamalar dikkate alınarak Nav2 parametrelerinden optimal ayarlar uygulanmıştır.

### 6.3. AMCL ile Harita Üzerinde Konumlandırma

Nav2 bileşeninin doğru işlemesi açısından ilk etapta haritada mevcut konum tespitine (Localization) ihtiyaç duyulmaktadır. Nav2 sistemiyle birlikte gelen AMCL (Adaptive Monte Carlo Localization) algoritması, 2D Lidar taramasını statik harita verisi ile parçacık filtresi (particle filter) marifetiyle eşleştirerek çalıştırılmıştır. Robot başlangıç evresinde ilk pozisyonu (Initial Pose) ayarlandıktan sonra, Lidar sinyalleri sayesinde anlık konumunu sürekli olarak güncelleyebilecek olgunluğa ulaştırılmıştır.

# 7. MESLEKİ, TEKNİK VE İŞ HAYATI KAZANIMLARI

Geride bırakılan bir aylık staj süresi, yalnızca pratik yazılım uygulamalarının ötesine geçmiş olup mesleki altyapı ve Ar-Ge kültürü anlamında değerli kazanımlar sağlamıştır.

### 7.1. Teknik ve Teorik Beceriler
- ROS2 ekosisteminde (Publish/Subscribe, Services, Actions, TF2 Hiyerarşisi) modern robotik yazılımların genel mimari tasarımı konusunda sağlam bir pratik altyapısı edinilmiştir.
- Sistemin (özel offsete sahip tricycle) klasik çözümlere tam uymaması üzerinden, matematiksel modelleme becerisine ve bu modellemeyi kaynak koda çevirme yöntemlerine dair kazanımlar elde edilmiştir.
- Robotik donanımların sensör tabanlı zayıf noktaları kavramsal olarak analiz edilmiş, EKF gibi olasılıksal yöntemlerin (probabilistic robotics) sapmaları gidermedeki etkisi gözlemlenmiştir. 
- Gerçekleme (Deployment) mekanizmalarına katkıda bulunacak şekilde Linux otomasyonları ve Dockerizasyon süreçlerine dair tecrübeler kazanılmıştır.

### 7.2. İş Çevresi, Süreç Yönetimi ve Takım Çalışması
- Modoya bünyesinde yürütülen Ar-Ge projeleri özelinde, "zaman", "donanım maliyeti", "işlemci yükü" ve "güvenilirlik" prensiplerinin ne derece optimize edilmesi gerektiği detaylı olarak kavranmıştır.
- Planlı toplantılar ve disiplinli takip yöntemleri sayesinde projelerin "tasarım", "test" ve "yayına alma" fazları adım adım gözlemlenmiştir.
- Süreçte elde edilen her problem, sistematik bir hata ayıklama (debugging) yaklaşımıyla çözümlenmiş ve araştırma bilincini kurumsal standartlara adapte etme fırsatı bulunmuştur.

# 8. SONUÇ

İlk bir aylık sürecin sonunda, baştan sona bir mobil robot ürün geliştirme yaşam döngüsü simülasyon düzeyinde somut bir çerçeveye oturtulmuştur. Kinematik zorlukların aşılması, farklı kontrolcülerin ayarlanması, ortamın haritalanması ve ileri düzey otonom seyrüsefer (Navigasyon) bileşenlerinin devreye alınmasına varan kritik süreçler başarıyla tamamlanmıştır. Raporlanan ay sonrasında, edinilen bu simülasyon ve yazılım altyapısının Modoya bünyesindeki fiziksel platformlar üzerine aktarılması ve gerçek dinamik ortam testlerinin yapılması (Deployment çalışmaları) hedeflenmektedir. Bu süreçte kazanılan hem robotik teoriler hem de mühendislik refleksleri, akademik eğitimin profesyonel sahadaki uygulamalarına dair büyük bir tecrübe kaynağı olmuştur.
