function pyenv_setup()
    % PYENV_SETUP Configura l'ambiente Python per MATLAB
    % Puntato direttamente al percorso rilevato in VS Code
    
    envName = 'labauto';
    
    % Determina il sistema operativo
    if ~ispc
        error('Questo script è stato configurato specificamente per il tuo ambiente Windows.');
    end
    
    % Percorso ESATTO dell'ambiente conda preso dalla tua immagine
    condaEnvRoot = 'C:\Users\anton\anaconda3\envs\labauto';
    
    % Verifica che il percorso esista effettivamente
    if ~exist(condaEnvRoot, 'dir')
        error('Impossibile trovare la cartella: %s', condaEnvRoot);
    end
    
    % Percorsi da aggiungere al PATH per far funzionare le librerie C/C++ di Python (come numpy, scipy)
    pathsToAdd = {
        condaEnvRoot;
        fullfile(condaEnvRoot, 'Library', 'mingw-w64', 'bin');
        fullfile(condaEnvRoot, 'Library', 'usr', 'bin');
        fullfile(condaEnvRoot, 'Library', 'bin');
        fullfile(condaEnvRoot, 'Scripts');
        fullfile(condaEnvRoot, 'bin');
        fullfile(condaEnvRoot, 'Library', 'site-packages', 'scipy.libs');
        fullfile(condaEnvRoot, 'Library', 'site-packages', 'pandas', '_libs', 'window');
        fullfile(condaEnvRoot, 'Library', 'site-packages', 'numpy');
        fullfile(condaEnvRoot, 'Library', 'site-packages', 'llvmlite', 'binding');
        fullfile(condaEnvRoot, 'Library', 'site-packages', 'h5py');
    };
    
    % Aggiungi i nuovi percorsi in cima al PATH corrente di Windows
    currentPath = strsplit(getenv('PATH'), ';')';
    newPath = [pathsToAdd; currentPath];
    newPath = unique(newPath, 'stable');
    setenv('PATH', strjoin(newPath, ';'));
    
    % Definisci l'eseguibile Python
    pyExec = fullfile(condaEnvRoot, 'python.exe');
    
    % Configura Python in MATLAB (usa pyenv per le versioni recenti di MATLAB)
    try
        % MATLAB R2019b o successivi
        pyenv('Version', pyExec);
    catch
        % Fallback per versioni più vecchie
        pyversion(pyExec);
    end
    
    fprintf('✓ Ambiente Python "%s" configurato correttamente!\n', envName);
    fprintf('  Eseguibile in uso: %s\n', pyExec);
end