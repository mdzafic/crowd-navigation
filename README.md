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
- Helbing i Molnár, *Social force model for pedestrian dynamics* (Physical Review E, 51(5), 1995): osnovni Social Force model.
- van den Berg, Guy, Lin, Manocha, *Reciprocal n-body collision avoidance* (Springer Tracts in Advanced Robotics, vol. 70, 2011): ORCA.
- Referentna implementacija ORCA-e: RVO2 biblioteka (`snape/RVO2`, GitHub), koristi se isključivo kao referenca za provjeru.

**`paralelizacija/`**: radovi o paralelnoj obradi simulacije gomile.
- Guy et al., *ClearPath: Highly Parallel Collision Avoidance for Multi-Agent Simulation* (SCA 2009): dinamičko raspoređivanje agenata, SIMD pristupi, ublažavanje divergencije.
- Bleiweiss, *Multi Agent Navigation on the GPU* (GDC 2009): raspored podataka na GPU-u (SoA), prostorni hashing, analiza iskorištenja GPU-a.
- Charlton, Gonzalez, Maddock, Richmond, *Fast Simulation of Crowd Collision Avoidance* (CGI 2019): GPU implementacija ORCA-e uz prostornu mrežu, problem balansa opterećenja.
- Skrzypczak i Czarnul, *Efficient parallel implementation of crowd simulation using a hybrid CPU+GPU high performance computing system* (Simulation Modelling Practice and Theory, 2022): OpenMP + CUDA implementacija.

**`prostorno-indeksiranje/`**: radovi o prostornim strukturama i traženju susjeda (Uniform Grid, cell lists, hashing na GPU-u).

**`linkovi-implementacije.md`**: linkovi na implementacije, dokumentaciju (OpenMP, CUDA, Thrust/CUB) i alate za profilisanje.

> Napomena: dio radova opisuje modele i tehnike drugačije od naše implementacije. Koriste se kao literatura za razumijevanje paralelizacije, balansa opterećenja i prostornih struktura, a ne kao direktna implementacijska osnova za sve dijelove projekta.

## Model kretanja

Model kretanja računa novu brzinu agenta iz cilja, susjeda i prepreka. Isti model koristi se u sekvencijalnoj, OpenMP i CUDA verziji, jer bi se u suprotnom ubrzanje od paralelizacije miješalo sa razlikom u količini posla, a ispravnost se ne bi mogla provjeriti poređenjem rezultata. Mijenja se samo implementacija prilagođena hardveru, ne matematika.

| | Social Force (Helbing) | ORCA (van den Berg et al.) |
|---|---|---|
| Ideja | sila ka cilju i sile odbijanja od agenata i zidova | za svakog susjeda jedno linearno ograničenje u prostoru brzina; rješava se linearno programiranje |
| Sudari | nema garancije, pri velikoj gustini moguća preklapanja | dizajnirana da izbjegava sudare (uz pretpostavku da svi agenti koriste isti algoritam) |
| Implementacija | jednostavnija | složenija (LP, prepreke kao segmenti, nemogući slučajevi) |
| Posao po agentu | zavisi od broja susjeda | zavisi od broja susjeda i grananja u LP-u |

**Radna pretpostavka:** Social Force kao glavni model, uz ORCA-u kao moguće proširenje.

## Inicijalni plan implementacije
 
Projekat se piše u C++20 uz CMake kao jedna zajednička jezgra sa tri izvršna dijela: sekvencijalnim, OpenMP i CUDA. Sva tri koriste isti model, iste scenarije i isti format izlaza, pa se rezultati i vremena mogu direktno porediti.
 
### Stanje simulacije
 
- **Agenti:** pozicija, brzina, cilj, radijus i željena brzina, pohranjeni kao SoA (zasebni nizovi `x`, `y`, `vx`, `vy`, `gx`, `gy`). Stanje se drži u dva primjerka (trenutno i sljedeće), pa svi agenti u jednom koraku čitaju isti ulaz i redoslijed obrade ne mijenja rezultat.
- **Prepreke:** zidovi i granice prostora kao skup dužina.
- **Prostorna mreža:** kvadratne ćelije stranice jednake radijusu interakcije `R`. Agenti su u niz indeksa sortiranom po ćelijama, uz zaseban niz početaka ćelija, pa je pretraga susjeda niz sekvencijalnih čitanja.
### Faza 1: sekvencijalna verzija
 
Redoslijed izrade, svaki korak se testira prije sljedećeg:
 
1. kretanje ka cilju bez interakcije,
2. naivno O(N²) izbjegavanje agenata, kao referenca ispravnosti,
3. zidovi,
4. prostorna mreža, uz provjeru da daje iste sile kao naivna verzija,
5. snimanje pozicija i referentnog izlaza (fiksan seed), pa označavanje verzije Git tagom.
Korak simulacije: izgradnja mreže, traženje susjeda u radijusu `R`, računanje nove brzine (sila ka cilju plus odbijanje od agenata i zidova), integracija uz ograničenje maksimalne brzine i, opcionalno, korekcija preklapanja. Računanje brzine je izdvojeno u zasebnu funkciju, pa se model može zamijeniti bez dodira mreže i paralelizacije.
 
### Faza 2: OpenMP
 
- paralelna petlja po agentima i paralelna izgradnja mreže (lokalni histogrami po nitima, pa prefix sum),
- poređenje `static`, `dynamic` i `guided` rasporeda na rijetkim i gustim scenarijima,
- sortiranje agenata po ćeliji radi lokalnosti keša, uz kontrolu false sharinga,
- opcionalno vektorizacija računanja sila (`omp simd`, AVX2).
### Faza 3: CUDA
 
- kernel za ćeliju svakog agenta, sortiranje po ćeliji (Thrust ili CUB), kernel za granice ćelija, kernel za sile i integraciju,
- podaci ostaju na GPU-u, a kopiranje na CPU služi samo za izlaz,
- analiza veličine bloka, coalescing-a, divergencije u gustim ćelijama i occupancy-ja, uz profilisanje alatom Nsight.
### Ulaz, izlaz i metrike
 
Ulaz je scenarij (generisan sa fiksnim seedom ili učitan iz datoteke). Izlaz su pozicije agenata svakih `k` koraka i konačno stanje, a 2D vizualizacija čita taj izlaz i nije dio mjerenja. Pratimo vrijeme po koraku (izgradnja mreže, susjedi, sile), broj agent-koraka u sekundi, ubrzanje i efikasnost, broj preklapanja i vrijeme evakuacije.

## Mjerenja

Predložene veličine skupova:

- Broj agenata N ∈ {1.000, 10.000, 100.000}, na GPU-u i više, u skladu sa memorijom.
- Broj niti ∈ {1, 2, 4, 8, ...}, u skladu sa dostupnim hardverom.
- Scenariji iz tabele iznad, uz različite gustine.

Porede se:

1. Sekvencijalna naivna verzija.
2. Sekvencijalna verzija sa prostornom mrežom.
3. OpenMP verzije (različiti rasporedi, sa i bez sortiranja agenata po ćeliji).
4. CUDA verzija.

Metrike:

- Vrijeme izgradnje mreže, vrijeme traženja susjeda i računanja brzine, ukupno vrijeme po koraku.
- Broj agent-koraka u sekundi.
- Ubrzanje: `S = T_seq / T_par`.
- Paralelna efikasnost: `E = S / broj_niti`.
- Strong scaling (isti N, više niti) i weak scaling (N raste sa brojem niti).
- Na GPU-u: vrijeme kernela i vrijeme prenosa podataka host-device mjere se odvojeno.
- Raspodjela broja susjeda po agentu (histogram) kao pokazatelj neravnomjernosti posla.

Ubrzanje paralelne verzije sa mrežom mjeri se u odnosu na **sekvencijalnu verziju sa mrežom**, kako bi se razdvojio doprinos paralelizacije od doprinosa same prostorne strukture.

Mjerenja se ponavljaju više puta, uz zagrijavanje, a uzima se medijan. Mjeri se samo simulacija (bez vizualizacije i učitavanja podataka).

## Izgradnja i pokretanje

> Naredbe će se dopuniti kada implementacija bude dostupna.

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Očekivani zahtjevi: C++ kompajler sa podrškom za OpenMP, CMake, CUDA Toolkit (za GPU verziju), Python 3 sa `matplotlib` (za vizualizaciju i grafikone).

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