# Zapisnik optimizacije: Crowd Navigation (ORCA)

## EXP-000 | 2026-11-15 | Okruženje i baseline repozitorija

**Verzija koda:** git commit `<6894f72>`

### Hardver
- CPU: Intel Core i5-13500H (Raptor Lake), 12 jezgara / 16 logičkih niti
  (4 P-jezgre s HT + 8 E-jezgara, prema specifikaciji procesora; osnovni takt 2,6 GHz)
- RAM: 24 GB (8 GB + 16 GB), 3200 MT/s, nesimetrična konfiguracija
- GPU: Intel Iris Xe (integrisana), drajver 31.0.101.4255; NVIDIA GPU nema
  → CUDA faza zahtijeva drugi računar ili alternativu

### Softver
- OS: Windows 11, build 22631
- Kompajler: GCC 16.2.0 (MSYS2 UCRT64), C:/msys64/ucrt64/bin/g++.exe
- CMake: 4.4.3, generator Ninja
- Build tip: Release (build\CMakeCache.txt); GCC defaultno -O3 -DNDEBUG
- Flagovi: CMAKE_CXX_FLAGS prazan, nema -march=native (cilja generički x86-64)
- Git: 2.49.0
- VTune: Intel VTune Profiler 2026.6.0

### Uslovi mjerenja
- Napajanje: Balanced (početno); za mjerenja prebaciti na High performance + punjač
- Napomene: laptop procesor (H serija), moguć termalni throttling; hibridna
  arhitektura P+E utiče na skaliranje OpenMP-a

## EXP-001 | 2026-11-15 | Baseline cijele simulacije (bez promjena)
**Verzija koda:** commit 6894f72 + izmijenjen src/main.cpp (CSV, min/max, avg_nb)

**Cilj:** raspodjela vremena po fazama, us/agent, šum mjerenja

**Parametri:** scenarij antipodal_disk, k=10, zagrijavanje 20 koraka, mjerenje koraka 21..120, 15 ponavljanja (medijan)

**Okruženje:** EXP-000; GCC 16.2.0 -O3 -march=native; serijski kod

**Rezultati (ms po koraku, medijan):**
| N | mreža | top-k | ORCA+pomak | ukupno | min | max | us/agent | avg_nb |
|---|---|---|---|---|---|---|---|---|
| 1000 | 0.003 | 0.755 | 0.170 | 0.924 | 0.911 | 0.981 | 0.924 | 10.00 |
| 5000 | 0.014 | 5.383 | 0.791 | 6.186 | 6.159 | 6.291 | 1.237 | 10.00 |
| 10000 | 0.030 | 10.187 | 1.458 | 11.667 | 11.636 | 11.843 | 1.167 | 10.00 |
| 20000 | 0.060 | 17.277 | 2.679 | 20.013 | 19.943 | 20.135 | 1.001 | 10.00 |

**Raspodjela (N=20000):** mreža 0,3%, top-k 86%, ORCA+pomak 13%.

**Šum:** unutar pokretanja ≤1,5%; između dva ranija pokretanja starog main.cpp do 3,8%.

**Zapažanja:**
- top-k dominira (82–88%); po Amdahlu ubrzanje samo ORCA dijela ≤ ~1,15x.
- "ORCA" faza uključuje i pomak/integraciju (faza 3 u Sim::step), jer nema posebne update faze.
- avg_nb = 10 = k: svi agenti zasićeni, pa avg_nb ne opisuje stvarnu gustoću.
- us/agent nije konstantno (0,92–1,24); uzrok nepoznat, vjerovatno broj kandidata.

**Zaključak:** prva meta je top-k faza. Za EXP-002 mjeriti broj kandidata/agent i VTune Hotspots s -g