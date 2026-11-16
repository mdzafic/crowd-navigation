# Paralelna simulacija navigacije gomile (Crowd Navigation)

> Projekat iz predmeta Paralelni računarski sistemi | Elektrotehnički fakultet Sarajevo | akademska 2026/27. godina

Cilj projekta je implementirati simulaciju kretanja velikog broja agenata (lokalno izbjegavanje sudara uz pronalaženje susjeda pomoću prostorne mreže), a zatim je paralelizovati i optimizovati na **CPU-u (OpenMP)** i na **grafičkoj kartici (CUDA)**. Težište nije na vizuelnom kvalitetu simulacije, nego na efikasnoj paralelizaciji, mjerenju ubrzanja i objašnjenju zašto ubrzanje nije idealno.

## Opis problema

Svaki agent ima poziciju, brzinu i cilj kretanja. Njegovo ponašanje mora uzeti u obzir statičke prepreke (zidove, granice prostora) i druge agente. Lokalno izbjegavanje sudara oslanja se na pronalaženje susjeda u neposrednoj okolini agenta, pa se umjesto poređenja svakog agenta sa svim ostalima (O(N²)) koristi **prostorna mreža** (Uniform Grid) koja smanjuje broj provjera.

Centralna poteškoća za paralelizaciju je **neravnomjerna raspodjela posla**: u gustim skupinama agent ima mnogo susjeda i radi znatno više posla nego agent u rijetko naseljenom dijelu. Ovo utiče na balans opterećenja niti na CPU-u i na divergenciju warp-ova na GPU-u.

## Ciljevi projekta

1. Odabir teme i pregled literature.
2. Sekvencijalna implementacija (referenca ispravnosti i performansi).
3. Paralelizacija na CPU-u (OpenMP), uz optimizaciju, a ne samo ispravan rad.
4. Paralelizacija na GPU-u (CUDA), uz analizu iskorištenja hardvera.
5. 2D vizualizacija kojom se pokazuje da algoritam radi ispravno.
6. Mjerenje i analiza: ubrzanje, efikasnost, skaliranje, uticaj gustine agenata.

## Dokumentacija

Folder `docs/` sadrži naučne radove, bilješke i linkove relevantne za projekat.

**`model/`**: radovi o modelima kretanja i izbjegavanja sudara.

**`paralelizacija/`**: radovi o paralelnoj obradi simulacije gomile.

**`prostorno-indeksiranje/`**: radovi o prostornim strukturama i traženju susjeda.

**`linkovi-implementacije.md`**: linkovi na implementacije, dokumentaciju (OpenMP, CUDA, Thrust/CUB) i alate za profilisanje.

> Napomena: dio radova opisuje modele i tehnike drugačije od naše implementacije. Koriste se kao literatura za razumijevanje paralelizacije, balansa opterećenja i prostornih struktura, a ne kao direktna implementacijska osnova za sve dijelove projekta.

## Model kretanja

Za izbjegavanje sudara koristi se **ORCA** (Optimal Reciprocal Collision Avoidance, van den Berg et al.). Za svakog susjeda agent dobija jedno linearno ograničenje u prostoru brzina, a nova brzina se bira rješavanjem linearnog programa tako da bude što bliža preferiranoj brzini ka cilju. Isti model koristi se u sekvencijalnoj, OpenMP i CUDA verziji, jer bi se u suprotnom ubrzanje od paralelizacije miješalo sa razlikom u količini posla, a ispravnost se ne bi mogla provjeriti poređenjem rezultata. Mijenja se samo implementacija prilagođena hardveru, ne matematika.
 
### Šta je implementirano
 
- **Navigacijsko polje:** Dijkstra sa 8 susjeda daje preferirani smjer prema cilju i penalizuje ćelije blizu zida.
- **Agent-agent ORCA ograničenja** za `k` najbližih susjeda, uz rješavanje linearnog programa za novu brzinu (validirano testovima).
- **Zidovi:** mapa se pretvara u segmente, a za segmente u dometu agenta generišu se statička ORCA ograničenja prije agent-agent ograničenja.
- **Sigurnosna mreža:** nakon ORCA integracije ostaje geometrijska provjera `blocked`; ona nije zamjena za zidna ograničenja.
- **Referenca ispravnosti:** RVO2 biblioteka koristi se isključivo za provjeru rezultata.
Pretraga segmenata je trenutno linearna kroz segmente mape. Za velike mape sljedeći korak je prostorni indeks segmenata po ćelijama, ali to ne mijenja model ograničenja.
 
## Implementacija
 
Projekat se piše u C++20 uz CMake kao jedna zajednička jezgra sa tri izvršna dijela: sekvencijalnim, OpenMP i CUDA. Sva tri koriste isti model, iste scenarije i isti format izlaza, pa se rezultati i vremena mogu direktno porediti.
 
### Stanje simulacije
 
- **Agenti:** pozicija, brzina, cilj, radijus i željena brzina, pohranjeni kao SoA (zasebni nizovi `x`, `y`, `vx`, `vy`, `gx`, `gy`). Stanje se drži u dva primjerka (trenutno i sljedeće), pa svi agenti u jednom koraku čitaju isti ulaz i redoslijed obrade ne mijenja rezultat.
- **Prepreke:** zidovi i granice prostora kao skup dužina, učitani iz ASCII mapa.
- **Prostorna mreža:** agenti su u nizu indeksa sortiranom po ćelijama, uz zaseban niz početaka ćelija, pa je pretraga susjeda niz sekvencijalnih čitanja. Veličina ćelije zavisi od parametra podjele (`subdiv`, trenutno 3, vidi EXP-005). <!-- TODO: provjeriti tačan opis parametra subdiv -->
### Korak simulacije
 
1. izgradnja mreže,
2. traženje `k` najbližih susjeda za svakog agenta (top-k),
3. ORCA: statička ograničenja zidova, pa agent-agent ograničenja, pa rješavanje linearnog programa za novu brzinu,
4. integracija pozicija uz ograničenje maksimalne brzine i `blocked` provjeru.
Računanje brzine je izdvojeno u zasebnu funkciju, pa se model može zamijeniti bez dodira mreže i paralelizacije.
 
### Planirano
 
**OpenMP:**
- paralelna petlja po agentima i paralelna izgradnja mreže (lokalni histogrami po nitima, pa prefix sum),
- poređenje `static`, `dynamic` i `guided` rasporeda na rijetkim i gustim scenarijima,
- sortiranje agenata po ćeliji radi lokalnosti keša, uz kontrolu false sharinga,
- opcionalno vektorizacija (`omp simd`, AVX2).
**CUDA:**
- kernel za ćeliju svakog agenta, sortiranje po ćeliji (Thrust ili CUB), kernel za granice ćelija, kernel za top-k i ORCA,
- podaci ostaju na GPU-u, a kopiranje na CPU služi samo za izlaz,
- analiza veličine bloka, coalescing-a, divergencije u gustim ćelijama i occupancy-ja, uz profilisanje alatom Nsight.


## Struktura repozitorija
 
| Folder / fajl | Sadržaj |
|---|---|
| `include/` | headeri (`agents`, `grid`, `map`, `metrics`, `navfield`, `orca`, `params`, `scenario`, `sim`; `.hpp`) |
| `src/` | implementacija (`grid`, `map`, `metrics`, `navfield`, `orca`, `main`; `.cpp`) |
| `tests/` | testovi: mreža, mapa, ORCA, simulacija i validacija |
| `maps/` | ASCII mape za benchmark po mapi |
| `scripts/` | pomoćne skripte (`run_bench.ps1` pokreće benchmark i dopisuje rezultate u CSV) |
| `results/` | rezultati mjerenja (`bench.csv`) |
| `docs/` | literatura i bilješke |
| `CMakeLists.txt` | build konfiguracija |
 
Generisani folderi (`build/`, `build-prof/`, `build-diag/`, `.vs/`) i izlaz alata za profilisanje (Intel VTune) nisu dio repozitorija i nalaze se u `.gitignore`.
 
## Izgradnja i pokretanje
 
Zahtjevi: C++ kompajler sa podrškom za C++20 i OpenMP, CMake, CUDA Toolkit (za GPU verziju, kad bude dostupna), Python 3 sa `matplotlib` (za grafikone). Za vizualizaciju se koristi biblioteka raylib, koju CMake preuzima sam. <!-- TODO: potvrditi -->
 
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```
 
Mjerenja imaju smisla samo u **Release** konfiguraciji.
 
### Testovi
 
```bash
ctest --test-dir build --output-on-failure
```
 
<!-- TODO: potvrditi da su testovi registrovani u CMake-u, inače navesti izvršne fajlove -->
 
### Benchmark
 
Benchmark bez mape koristi scenarij antipodalnog diska (broj u argumentu je broj ponavljanja):
 
```bash
./build/crowd_bench
./build/crowd_bench 10
```
 
Benchmark jedne ASCII mape koristi `--map <fajl> [N] [ponavljanja]`:
 
```bash
./build/crowd_bench --map maps/labirint.txt
./build/crowd_bench --map maps/vrata.txt 500 5
```
 
Bez `N` koriste se četiri veličine proporcionalne mapi: `slobodno/8`, `slobodno/4`, `slobodno/2` i `slobodno`. Ako navedeš `N`, benchmarkuje se samo ta jedna veličina. Za svaku mapu pokreće se isti benchmark: 20 koraka zagrijavanja i 100 mjerenih koraka.
 
### Benchmark sa CSV izlazom (Windows, PowerShell)
 
```powershell
.\scripts\run_bench.ps1 -Label "naziv-eksperimenta" -Repeats 7
```
 
Skripta pokreće `build\crowd_bench.exe` i rezultate sprema u `results\bench.csv` uz zadanu oznaku (`--label`). Za svaku optimizaciju koristi drugu oznaku da se rezultati ne miješaju.
 
## Mjerenja
 
Veličine skupova:
 
- Broj agenata N ∈ {1.000, 5.000, 10.000, 20.000} u trenutnim mjerenjima; planirano i 100.000, na GPU-u i više, u skladu sa memorijom.
- Broj niti ∈ {1, 2, 4, 8, ...}, u skladu sa dostupnim hardverom.
- Scenariji: antipodalni disk i ASCII mape, uz različite gustine.
Porede se:
 
1. Sekvencijalna naivna verzija.
2. Sekvencijalna verzija sa prostornom mrežom.
3. OpenMP verzije (različiti rasporedi, sa i bez sortiranja agenata po ćeliji).
4. CUDA verzija.
Metrike:
 
- Vrijeme izgradnje mreže, vrijeme top-k pretrage, vrijeme ORCA-e i ukupno vrijeme po koraku.
- Vrijeme po agentu (`us/agent`) i prosječan broj susjeda (`avg_nb`).
- Broj agent-koraka u sekundi.
- Ubrzanje: `S = T_seq / T_par`.
- Paralelna efikasnost: `E = S / broj_niti`.
- Strong scaling (isti N, više niti) i weak scaling (N raste sa brojem niti).
- Na GPU-u: vrijeme kernela i vrijeme prenosa podataka host-device mjere se odvojeno.
- Raspodjela broja susjeda po agentu (histogram) kao pokazatelj neravnomjernosti posla.
Ubrzanje paralelne verzije sa mrežom mjeri se u odnosu na **sekvencijalnu verziju sa mrežom**, kako bi se razdvojio doprinos paralelizacije od doprinosa same prostorne strukture.
 
Mjerenja se ponavljaju više puta (7 ponavljanja), uz zagrijavanje od 20 koraka (mjere se koraci 21 do 120), a uzima se medijan. Mjeri se samo simulacija (bez vizualizacije i učitavanja podataka).

## Pojmovi i skraćenice

| Pojam | Značenje |
|---|---|
| Crowd Navigation | Navigacija većeg broja agenata prema ciljevima uz izbjegavanje prepreka i drugih agenata |
| Agent | Entitet sa pozicijom, brzinom i ciljem |
| Social Force | Model u kojem kretanje agenta određuju sile privlačenja ka cilju i odbijanja od agenata i zidova |
| ORCA | Optimal Reciprocal Collision Avoidance, izbjegavanje sudara pomoću linearnih ograničenja u prostoru brzina |
| Velocity Obstacle (VO) | Skup brzina koje bi dovele do sudara |
| Uniform Grid | Prostorna mreža jednakih ćelija za traženje susjeda |
| Counting sort / prefix sum | Algoritmi za gradnju mreže raspoređivanjem agenata po ćelijama |
| Load imbalance | Neravnomjerna raspodjela posla među nitima |
| Divergencija warp-a | Niti istog warp-a na GPU-u izvršavaju različite putanje i čekaju jedna drugu |
| Coalescing | Spajanje pristupa globalnoj memoriji GPU-a u jednu transakciju |
| Occupancy | Iskorištenost multiprocesora na GPU-u |
| False sharing | Usporenje kada više niti piše u različite elemente iste cache linije |
| SoA / AoS | Structure of Arrays / Array of Structures, rasporedi podataka u memoriji |
| OpenMP | Tehnologija za paralelizaciju pomoću više niti |
| CUDA | NVIDIA platforma za programiranje GPU-a |
| SIMD / AVX2 | Obrada više podataka jednom instrukcijom / 256-bitni skup vektorskih instrukcija |
| Data race | Nesinhronizovan pristup zajedničkim podacima iz više niti |
| Speedup | Odnos vremena sekvencijalnog i paralelnog izvršavanja |
| Parallel efficiency | Odnos ubrzanja i broja korištenih niti |
| Strong / weak scaling | Skaliranje pri konstantnoj / proporcionalno rastućoj veličini problema |