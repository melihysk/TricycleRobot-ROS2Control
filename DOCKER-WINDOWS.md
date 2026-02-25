# forklift – Windows’ta Docker ile Çalıştırma

Bu belge, projeyi Windows’ta çalıştırmak için **iki yöntemi** adım adım anlatır:

1. **Yöntem 1:** Hazır Docker image’ını oluşturup karşı tarafa vermek (derleme karşı tarafta yok).
2. **Yöntem 2:** Proje dosyalarını verip karşı tarafta image’ı kendilerinin derlemesi.

Her iki durumda da **Windows’ta Docker Desktop (WSL2)** ve isteğe bağlı **GUI / GPU** ayarları gerekir; aşağıda hepsi açıklanıyor.

---

## Windows’ta Ortak Gereksinimler

Karşı tarafta (Windows kullanıcısında) şunlar kurulu olmalı:

| Gereksinim | Açıklama |
|------------|----------|
| **Windows 10/11** | 64-bit, güncel sürüm. |
| **WSL2** | “Windows Subsystem for Linux” 2. Yüklü değilse: PowerShell (Yönetici): `wsl --install` |
| **Docker Desktop** | [Docker Desktop for Windows](https://docs.docker.com/desktop/install/windows-install/) indirilip kurulmalı. Ayarlardan **“Use the WSL 2 based engine”** seçili olmalı. |
| **NVIDIA GPU (isteğe bağlı)** | Gazebo/RViz hızı için. [NVIDIA sürücüsü](https://www.nvidia.com/Download/index.aspx) + WSL2 için [CUDA on WSL](https://docs.nvidia.com/cuda/wsl-user-guide/index.html) / Docker Desktop’ta “GPU support” kullanılabilir. |

**Önemli:** GUI (RViz2, Gazebo) için komutları **WSL2 içinden** (ör. Ubuntu terminali) çalıştırmak gerekir; böylece WSLg ile pencere Windows’ta açılır. PowerShell/CMD’den çalıştırırsanız DISPLAY ayarı olmaz ve GUI açılmayabilir.

---

# Yöntem 1: Hazır Image’ı Vererek Çalıştırma

Bu yöntemde **siz** Linux’ta image’ı build edip bir dosyaya (tar) kaydediyorsunuz. Karşı taraf bu dosyayı alıp Windows’ta Docker’a yüklüyor; **hiç `docker build` yapmıyor**.

## 1.1 Sizin Yapacaklarınız (Linux’ta)

### Adım 1: Image’ı oluşturun

Proje kökünde (`/home/melih/Desktop/forklift`):

```bash
docker compose build
# veya
docker build -t forklift:latest .
```

### Adım 2: Image’ı dosyaya aktarın (export)

```bash
docker save -o forklift-image.tar forklift:latest
```

Bu komut `forklift-image.tar` adında (birkaç GB olabilir) bir dosya üretir.

### Adım 3: Dosyayı karşı tarafa iletin

- USB bellek, ağ paylaşımı, WeTransfer, Google Drive vb. ile `forklift-image.tar` dosyasını Windows kullanıcısına verin.
- Dosya büyük olduğu için sıkıştırabilirsiniz (isteğe bağlı):
  ```bash
  gzip -k forklift-image.tar
  ```
  Bu durumda karşı tarafa `forklift-image.tar.gz` verilir; Windows’ta açarken önce `gzip` ile açması gerekir (WSL veya 7-Zip ile).

---

## 1.2 Karşı Tarafın Yapacakları (Windows’ta)

### Adım 1: Docker Desktop’ı kurup çalıştırın

- Docker Desktop yüklü ve WSL2 backend’i açık olsun.
- İlk çalıştırmada WSL2 kurulumu bitene kadar bekleyin.

### Adım 2: Image dosyasını uygun bir yere kopyalayın

- `forklift-image.tar` (veya `.tar.gz` ise açılmış hali `forklift-image.tar`) dosyasını örneğin `C:\Users\Kullanici\forklift\` gibi bir klasöre koyun.
- Sıkıştırılmışsa (`.tar.gz`), 7-Zip veya WSL’de `gunzip forklift-image.tar.gz` ile açın.

### Adım 3: Image’ı Docker’a yükleyin

**WSL2 terminali** açın (Windows’ta “Ubuntu” veya “WSL” aramak yeterli). Sonra:

```bash
# Dosyanın bulunduğu dizine gidin (Windows sürücüsü genelde /mnt/c/ altında)
cd /mnt/c/Users/Kullanici/forklift   # Kendi kullanıcı adınıza göre düzeltin

# Image'ı yükleyin
docker load -i forklift-image.tar
```

Çıktıda `Loaded image: forklift:latest` benzeri bir satır görmeliler.

### Adım 4: Konteyneri çalıştırın (GUI için WSL’den)

GUI (RViz2, Gazebo) için komutun **WSL2 içinden** çalışması gerekir. Aynı WSL terminalinde:

```bash
# X11/display WSLg ile gelir; bazen açıkça vermek gerekebilir
export DISPLAY=:0

docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --network host \
  forklift:latest \
  bash
```

Konteyner içinde:

```bash
ros2 launch offset_tricycle_robot tricycle_robot_sim.launch.py
```

**Not:** Windows’ta `network_mode: host` tam Linux’taki gibi çalışmaz; gerekirse `-p 11311:11311` gibi port eşlemeleri eklenebilir. Çoğu simülasyon senaryosu için yukarıdaki hali yeterli olur.

### GPU kullanacaklarsa (NVIDIA)

- NVIDIA sürücüsü + WSL2 için GPU desteği kurulu olmalı.
- Docker Desktop’ta: Settings → Resources → WSL Integration → “Enable GPU support” (varsa).
- Çalıştırma örneği:

```bash
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all,graphics,utility,compute \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --network host \
  --gpus all \
  forklift:latest \
  bash
```

---

# Yöntem 2: Proje Dosyalarını Vererek Çalıştırma

Bu yöntemde **tüm proje** (kaynak kod, Dockerfile, docker-compose) karşı tarafa verilir. Karşı taraf kendi ortamında image’ı kendisi build eder; böylece güncellemeleri çekip tekrar build etme esnekliği olur.

## 2.1 Sizin Yapacaklarınız (Linux’ta)

### Adım 1: Projeyi paylaşılabilir hale getirin

- **Seçenek A – Git:** Projeyi Git’e push ediyorsanız, karşı tarafa repo linkini (veya erişim haklarını) verin. Clone edecekler.
- **Seçenek B – Arşiv:** Derleme çıktılarını göndermeyin; sadece kaynak ve konfigürasyon yeterli:
  ```bash
  cd /home/melih/Desktop
  tar --exclude='forklift/build' \
      --exclude='forklift/install' \
      --exclude='forklift/log' \
      --exclude='forklift/.git' \
      -cvf forklift-src.tar forklift
  gzip forklift-src.tar
  ```
  Karşı tarafa `forklift-src.tar.gz` verin.

### Adım 2: Kısa talimatları iletin

Karşı tarafa şunu söyleyin: “DOCKER-WINDOWS.md dosyasındaki **Yöntem 2 – Karşı taraf** bölümünü takip edin.”

---

## 2.2 Karşı Tarafın Yapacakları (Windows’ta)

### Adım 1: Projeyi alıp açın

- **Git ile:**  
  WSL2 veya Git Bash’te:  
  `git clone <repo-url>`  
  Sonra `cd forklift` (veya repo adı ne ise).
- **Arşiv ile:**  
  `forklift-src.tar.gz`’yi bir klasöre kopyalayıp WSL’de açın:
  ```bash
  cd /mnt/c/Users/Kullanici/Desktop   # Örnek
  tar -xzf forklift-src.tar.gz
  cd forklift
  ```

### Adım 2: Docker Desktop’ı çalıştırın

Docker Desktop açık ve WSL2 kullanıyor olsun.

### Adım 3: Image’ı build edin

**WSL2 terminalinde** proje kökünde:

```bash
cd /mnt/c/Users/.../forklift   # Proje yolunu kendinize göre yazın

docker compose build
# veya
docker build -t forklift:latest .
```

İlk build uzun sürebilir (ROS/Gazebo indirmeleri ve derleme).

### Adım 4: Konteyneri çalıştırın

Yine **WSL2 içinden** (GUI için):

```bash
export DISPLAY=:0

docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --network host \
  forklift:latest \
  bash
```

Konteyner içinde:

```bash
ros2 launch offset_tricycle_robot tricycle_robot_sim.launch.py
```

İsterseniz proje kökünde `docker-compose.yml` ile de çalıştırabilirler; ancak Windows’ta `/dev/dri`, `/dev/input` ve `deploy.resources.reservations.devices` (NVIDIA) tam desteklenmeyebilir. En garanti yol, yukarıdaki `docker run` ile sadece `DISPLAY` ve `network host` kullanmaktır. GPU gerekirse “Yöntem 1”deki gibi `--gpus all` ve NVIDIA ortam değişkenlerini ekleyebilirler.

---

# Özet Tablo

| | Yöntem 1: Hazır image | Yöntem 2: Proje dosyaları |
|--|------------------------|----------------------------|
| **Siz** | `docker compose build` → `docker save -o forklift-image.tar forklift:latest` → dosyayı iletin | Projeyi Git veya arşivle (build/install/log hariç) iletin |
| **Karşı taraf** | `docker load -i forklift-image.tar` → `docker run ... forklift:latest bash` | Projeyi aç → `docker compose build` veya `docker build -t forklift:latest .` → aynı `docker run` |
| **Avantaj** | Karşı tarafta derleme yok, hızlı başlama | Güncelleme ve tekrar build kolay, tam kaynak erişimi |
| **Dezavantaj** | Image dosyası büyük; güncelleme için yeni image gerekir | İlk build uzun ve internet gerekir |

---

# Olası Sorunlar (Windows)

1. **Pencere açılmıyor (RViz/Gazebo)**  
   Komutları mutlaka **WSL2 terminalinden** çalıştırın; `DISPLAY=:0` ve `-v /tmp/.X11-unix:/tmp/.X11-unix` kullanın. Windows 11 ve güncel WSL2’de WSLg varsayılan gelir.

2. **“Cannot connect to X server”**  
   WSL’de: `export DISPLAY=:0` yapıp tekrar `docker run` deneyin. Gerekirse Windows’ta bir X sunucusu (VcXsrv, X410) kurup DISPLAY’i buna yönlendirin.

3. **Joystick çalışmıyor**  
   `/dev/input` Windows’ta doğrudan yok. Joystick USB’yi WSL2’ye yönlendirmek veya Windows’ta ayrı bir sürücü/uygulama kullanmak gerekebilir; proje dokümantasyonunda ayrıca ele alınabilir.

4. **GPU kullanılmıyor**  
   NVIDIA + WSL2 + Docker Desktop “GPU support” etkin olmalı; sürücü ve Docker sürümlerini güncel tutun.

Bu adımlarla hem **hazır image** hem de **proje dosyaları** yoluyla Windows’ta forklift simülasyonunu Docker ile çalıştırmak mümkündür.
