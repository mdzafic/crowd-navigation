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
**Verzija koda:** commit CROWD_DIAG brojači, opcija --n

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
**Verzija koda:** commit (izmjena petlje faze 2 u Sim::step), baseline 6894f72
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

## EXP-004 | 2026-11-16 | Odabir k najmanjih: k prolaza traženja minimuma (NEGATIVAN REZULTAT)

**Verzija koda:** commit (izmjena koraka 2 petlje faze 2), prethodna: EXP-003

**Hipoteza:** preostali Bad Speculation (32%) potiče iz `insert_neighbor`; fiksni predvidivi prolazi ga uklanjaju.

**Promjena:** zamjena umetanja sa k prolaza traženja minimuma preko m ≈ 56 kandidata (`cmov`, strogo `<`), isti redoslijed pri jednakim `d2`.

**Parametri:** `antipodal_disk`, k = 10, zagrijavanje 20, mjerenje 21..120, 15 ponavljanja, medijan; build/Release `-O3 -march=native`.

**Rezultati (ukupno ms, run a):**

| N | EXP-003 | EXP-004 | Promjena |
|---:|---:|---:|---:|
| 1000 | 0.651 | 0.772 | +19% |
| 5000 | 4.361 | 5.759 | +32% |
| 10000 | 8.334 | 10.679 | +28% |
| 20000 | 14.453 | 17.974 | +24% |

Run b: N = 1000 i 20000 potvrđuju; redovi 5000 i 10000 pokvareni šumom (max 12.9 i 21.9 ms).

**VTune uarch (N = 20000):**

| Metrika | EXP-003 | EXP-004 |
|---|---:|---:|
| Elapsed | 8.575 s | 10.655 s |
| Bad Speculation | 32.2% | 11.6% |
| Retiring | 48.3% | 50.7% |
| Back-End Bound | 7.5% | 31.8% (Core Bound 25.8%) |
| Instructions retired | 109.6 G | 142.7 G (+30%) |
| CPI | 0.334 | 0.320 |
| Vector Capacity Usage | — | 6.2% |

**Zaključak:** hipoteza odbačena. Grananje je uklonjeno (Bad Speculation -20.6 p.p.), ali ~560 poređenja po agentu naspram ~27 umetanja daje 30% više instrukcija; kod prelazi iz branch-bound u compute-bound (Core Bound, Port 6). Petlja arg-min nije se auto-vektorizirala (Vector Capacity 6.2%).

**Odluka:** vraćena EXP-003 verzija. Sljedeće: EXP-005, smanjenje broja kandidata (ćelije R/2, 5x5).

---

## EXP-005 | 2026-11-16 | Finija rešetka (ćelije R/s, pretraga (2s+1)²)

**Verzija koda:** baseline poređenja: EXP-003 (`ea9f2b7`).

**Hipoteza:** manje kandidata (pogodak rešetke 35% → 50%+) smanjuje `d2`/sabijanje; broj umetanja (~27/agent) ostaje, pa se očekuje skroman dobitak.

**Promjena:** Grid parametar `subdiv` (`R_ = R/subdiv`, `reach_ = subdiv`); mjereno za s = 1, 2, 3 preko `CROWD_SUBDIV`.

**Rezultati (ukupno ms, N = 20000):**

| Subdiv (s) | Ukupno vrijeme (ms) |
|---:|---:|
| 1 | 14.24 |
| 2 | 13.42 |
| 3 | 13.06 |

**Top-k rezultati:**

| Subdiv (s) | Top-k (ms) |
|---:|---:|
| 1 | 11.51 |
| 2 | 10.69 |
| 3 | 10.34 |

Ubrzanje za s = 3: 1.06–1.11x (1.09x pri N = 20000).

Mreža: 0.059 → 0.068 ms (zanemarivo). ORCA nepromijenjena (~2.67 ms).

**Kontrola:** s = 1 = EXP-003 (14.24 / 14.29 ms).

**Hotspots nakon EXP-005 (N = 20000):**

| Funkcija | Udio vremena |
|---|---:|
| `insert_neighbor` | 41.4% |
| `Sim::step` | 16.0% |
| lambda (`d2`/upis) | 11.3% |
| `for_each_candidate` | 5.5% |
| `linearProgram1` | 4.0% |

**Zaključak:** dobit 6–11%, granično ispod praga 1.1x. Zadržano s = 3 (najbrži, bez troška).

**Serijska faza:** ukupno baseline 20.01 ms → 13.06 ms pri N = 20000 = 1.53x.

---

## EXP-006 | 2026-11-16 | k najbližih: obilazak od najbliže ćelije + smanjivanje praga

**Verzija koda:** baseline poređenja: EXP-005 (`2c8c6db`); izmjene u `include/grid.hpp`, `src/grid.cpp`, `src/sim.cpp`, `tests/test_grid.cpp`.

**Hipoteza:** kandidati se obilaze red po red (nasumično po udaljenosti), pa se top-k stalno prepisuje (~27 umetanja za 10 mjesta) i uvijek se pregleda svih 49 ćelija. Ako se ćelije obilaze od najbliže i prag nakon punjenja top-k spusti sa R² na d2 k-tog susjeda (kao u RVO2 kd-stablu), odsijecaju se cijele ćelije, a većina preostalih kandidata se odbaci jednim predvidivim poređenjem.

**Promjena:**
- Nova metoda `Grid::k_nearest(self, R, k, ids, d2s)`; faza 2 u `Sim::step` je jedan poziv po agentu (`insert_neighbor` i baferi kandidata uklonjeni).
- Ćelije prozora (2s+1)² obilaze se prsten po prsten (unutar prstena po udaljenosti centra).
- Za svaku ćeliju računa se donja granica udaljenosti (po osi, rubne ćelije beskonačne prema van zbog clampanih agenata, mala rezerva `slack_` za zaokruživanje); ćelija se preskače ako je granica > prag. Cijeli prsten se prekida ako je njegova minimalna granica > prag.
- `build()` dodatno puni `sx_`, `sy_` (pozicije u redoslijedu `sorted_ids_`) i `pos_of_` → uzastopno čitanje bez indirekcije.
- Poredak pri jednakim d2 razrješava se po poziciji u `sorted_ids_`, što je tačno poredak stare petlje → izlaz bit-identičan.
- `CROWD_SUBDIV` se sada stvarno koristi (podrazumijevano 3).

**Parametri:** `antipodal_disk`, k = 10, zagrijavanje 20, mjerenje 21..120, 9 ponavljanja, medijan; Release `-O3 -march=native`. Stara i nova verzija mjerene naizmjenično (stara, nova, stara, nova).

**Ispravnost:**
- Dump pozicija i lista susjeda u svakom koraku (N = 5000 / 200 koraka, N = 20000 / 60 koraka): identičan hash stare i nove verzije.
- `test_grid`: `k_nearest` naspram brute-force top-k za k = 1, 10, 64 i subdiv = 1..4, uključujući agente van granica svijeta; svih 6 testova prolazi.

**Opterećenje po agentu po koraku (N = 20000, s = 3, CROWD_DIAG):**

| | EXP-005 | EXP-006 |
|---|---:|---:|
| Pregledanih kandidata | ~97* | 24.2 |
| Umetanja u top-k (min. 10) | ~26.6* | 13.8 |

\* Nije direktno izmjereno za EXP-005: kandidati su procjena (49 ćelija × gustoća, iz 158.7 kandidata na 9 ćelija u EXP-002), umetanja su vrijednost iz EXP-002 (s = 1); redoslijed obilaska je isti, pa se broj umetanja ne mijenja bitno.

Za poređenje, EXP-006 sa s = 1: 63.1 kandidat i 19.7 umetanja; s = 2: 32.9 i 16.5.

**Rezultati (ms/korak, medijan, N = 20000, dva naizmjenična pokretanja):**

| | EXP-005 | EXP-006 |
|---|---:|---:|
| top-k | 32.70 / 32.00 | 17.08 / 19.20 |
| ORCA (kontrola) | 10.04 / 9.80 | 8.19 / 9.02 |
| ukupno | 42.93 / 42.06 | 25.54 / 28.51 |

Ostale veličine (run 1): N = 1000 ukupno 2.46 → 1.46 ms, N = 5000 17.70 → 6.82 ms, N = 10000 35.95 → 13.03 ms.

**Izbor subdiv (EXP-006, N = 20000, ukupno ms):** s = 1: 27.92, s = 2: 25.49, s = 3: 28.19, s = 4: 26.70 — razlike unutar šuma; zadržano s = 3.

**Šum:** mjerenja su ~2.5x sporija od EXP-001..005 (napajanje vjerovatno Balanced, ne High performance). Stara verzija je imala jake skokove (N = 10000: top-k 27.5 ms naspram 18.3 ms u ranijem pokretanju; max do 64 ms), pa su apsolutni brojevi nepouzdani. **Ponoviti po protokolu iz EXP-000** (High performance + punjač, 15 ponavljanja) prije konačnog upisa ubrzanja.

**Zaključak:** hipoteza potvrđena. Kandidata 4x manje, umetanja 2x manje; top-k približno 1.7–1.9x brže, ukupno približno 1.5–1.7x (procjena, vidi šum). Top-k je sada blizu donje granice (13.8 umetanja za 10 susjeda), a ORCA čini ~1/3 vremena.

**Sljedeće:** ponoviti mjerenje u čistim uslovima i VTune uarch; zatim paralelizacija (OpenMP) — `k_nearest` je `const` i bez dijeljenih bafera, pa je petlja po agentima nezavisna. Opciono: kvalitet simulacije za k = 10 naspram k = 15–20.
