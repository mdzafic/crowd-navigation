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

## EXP-002 | 2026-11-15 | Dijagnostika opterećenja i VTune profil top-k faze (bez optimizacije)
**Verzija koda:** commit <hash> + CROWD_DIAG brojači, opcija --n

**Parametri:** antipodal_disk, N=20000, k=10, 3 ponavljanja, 120 koraka (VTune: 5 ponavljanja)

**Opterećenje po agentu po koraku:** kandidata 158,7 (uklj. sebe), u radijusu 55,9, ubačenih 26,6
- pogodak rešetke 35,4% (teorija π/9 = 34,9%)
- ubačeno 26,6 ≈ očekivanih 27,2 za nasumičan redoslijed (k + k·ln(n/k))
- nakon odbacivanja ostaje 10 od ~56 susjeda (~82% odbačeno)

**VTune Hotspots (build bez -g):** func@0x140005090 85,2% (vjerovatno Sim::step s inlinovanim top-k); ORCA funkcije ukupno ~14,4%

**VTune uarch:** Retiring 27,3%, Front-End 17,5%, Bad Speculation 46,0% (Branch Mispredict 48,2%), Back-End 9,3%; CPI 0,622; takt 4,4 GHz

**Zaključak:** top-k je branch-bound (ne memory-, ne compute-bound). Grananja: filtar radijusa (35% pogodaka), test odbacivanja u insert_neighbor (~50%), petlja pomicanja.

**Ispravke:** dvostruko računanje udaljenosti vjerovatno spojeno od kompajlera (nije prioritet); raniji FP Scalar/Vector nalazi ne odnose se na ovaj kod.

**Hipoteza za EXP-003:** bezgranično sabijanje kandidata smanjuje Bad Speculation; izlaz bit-identičan.

## EXP-003 | 2026-11-15 | Branchless filtar radijusa u top-k fazi
**Verzija koda:** commit <hash> (izmjena petlje faze 2 u Sim::step), baseline 6894f72
**Hipoteza:** grananje d2<=R2 (35% pogodaka, nepredvidivo) uzrokuje Bad Speculation; bezgranično sabijanje ga uklanja.
**Promjena:** d2 za sve kandidate, upis u lokalni bafer uvijek, brojač m povećan maskom;
insert_neighbor i redoslijed kandidata nepromijenjeni.
**Parametri:** antipodal_disk, k=10, zagrijavanje 20, mjerenje 21..120, 15 ponavljanja, medijan

**Rezultati (ms/korak, medijan, run b):**

| N | ukupno prije | ukupno poslije | ubrzanje | top-k prije → poslije |
|---|---|---|---|---|
| 1000 | 0.924 | 0.651 | 1.42x | 0.755 → 0.487 |
| 5000 | 6.186 | 4.361 | 1.42x | 5.383 → 3.575 |
| 10000 | 11.667 | 8.334 | 1.40x | 10.187 → 6.861 |
| 20000 | 20.013 | 14.453 | 1.38x | 17.277 → 11.715 |
ORCA nepromijenjena (~2.7 ms pri N=20000), što je kontrolni signal.

**Stabilnost:** razlika run a/b ≤ 1.2%, raspon unutar pokretanja ≤ 2.6%; jedan outlier 36.55 ms
(N=20000, run a), medijan nepromijenjen. Raniji pokvareni run (N=10000) bio je vanjski šum.

**VTune uarch (N=20000, 5 ponavljanja):**

| | prije | poslije |
|---|---:|---:|
| Elapsed | 12.009 s | 8.575 s |
| CPI | 0.622 | 0.334 |
| Retiring | 27.3% | 48.3% |
| Bad Speculation | 46.0% | 32.2% (Branch Mispredict 31.6%) |
| Front-End Bound | 17.5% | 12.0% |
| Back-End Bound | 9.3% | 7.5% |
| Instructions retired | 82.65 G | 109.59 G (+33%) |

**Zaključak:** ukupno ~1.4x, top-k -32..-36%. Ubrzanje nastaje jer se broj ciklusa smanjio 29% iako je
instrukcija 33% više: IPC 1.61 → 2.99, Bad Speculation -13.8 p.p. Hipoteza potvrđena.
Ostaje 32% Bad Speculation, uglavnom u insert_neighbor.

**Sljedeće:** EXP-004, odabir k najmanjih bez grananja.