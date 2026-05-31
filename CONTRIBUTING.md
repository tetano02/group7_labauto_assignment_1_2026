# Guida alla navigazione della repository

Questo file è utilizzato come guida per il docente, per rendere più semplice capire cosa è stato aggiunto dal gruppo, quali file sono stati modificati rispetto al branch `master` originale e dove cercare le parti principali del lavoro.

**Nota:** [Qui](https://docs.google.com/document/d/1mMn_XgthogLTIwmd0kHkKxPk2QxaSKGnX9Dktu6YbfQ/edit?usp=sharing) è possibile trovare il workflow con i parametri dettagliati.

## Struttura della repository

### Modelli identificati

I modelli identificati sono stati salvati in una cartella dedicata:

```text
data/models/
```
L'idea di base era che, in caso di più modelli validi e confrontabili, fosse più semplice cambiare i modelli di riferimento. Il collegamento a questo percorso è stato introdotto in ogni script al fine di caricare i modelli identificati per validazione e tuning.

**Nota**: I primi modelli identificati, definti con id `000`, sono risultati validi e dunque non è stato necessario aggiungere ulteriori modelli, ma la struttura è stata mantenuta per chiarezza e modularità.

### Script aggiunti o modificati

Sono stati utilizzati i file già disponbili andando a modificare i parametri già presenti come indicato nel report del progetto.

Gli script modificati o aggiunti sono:

- `robot_simulation.py`
  In questa versione sono state applicate due modifiche:
  - stampa il tempo totale di simulazione;
  - è possibile cambiare la traiettoria modificando `program_name` da `trajectory` a `trajectory_FF`, che è una traiettoria più complessa usata per identificazione e tuning. Di conseguenza lo script decide se usare il modello con o senza vasi modificando `xml_file` da `model.xml` a `model_without_vases.xml`.

- `visual_simulation_validation_chirp.py`
  Riproduce graficamente un esperimento chirp già salvato, utile per verificare visivamente se il moto rimane coerente o se compaiono problemi di finecorsa.

- `taratura_manuale_posizione.mlx`
  Script usato per calcolare il parametro proporzionale esterno a partire dal processo interno in retroazione con integratore. In seguito abbiamo preferito un approccio più diretto per separare le bande proporzionali, ma abbiamo comunque lasciato questo script perché è stato utile durante lo sviluppo.

- `taratura_ottima.mlx`
  È stato aggiunta la possibilità di inserire filtri passa-basso. Ciò non è stato applicato anche nella taratura manuale perchè era già stata preferita la taratura ottima.

- `control_config.yaml`
  Sono stati aggiunti per ogni giunto filtro passa-basso e notch, se utilizzati. È stato aggiunto il modello dinamico per l'utilizzo del feed-forward dinamico.

## Dati generati e file pesanti

Durante l'esecuzione degli script Python, nuovi risultati vengono generati in:

```text
gantry_portal_sea_soft/tests/
```

Questi dati sono stati aggiunti al `.gitignore` per evitare di appesantire la repository, ma sono comunque disponibili localmente e possono essere rigenerati eseguendo gli script Python.

### Conclusioni

In conclusione, è stato mantenuto l'approccio originale fornito dalla repository originale e integrato con i file forniti dal docente. In aggiunta sono stati modificati o aggiunti alcuni script per adattarli alle esigenze del progetto.

### Autori

- Stefano Agnelli
- Antonio Di Filippo
- Wen Wen Sun
