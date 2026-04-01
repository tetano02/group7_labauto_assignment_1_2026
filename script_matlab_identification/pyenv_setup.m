function pyenv_setup(envName)
    % PYENV_SETUP Configura l'ambiente Python per MATLAB
    %
    % Sintassi:
    %   pyenv_setup()           % Usa 'labauto' come default
    %   pyenv_setup('myenv')    % Usa l'ambiente specificato
    %
    % Input:
    %   envName (char/string) - Nome dell'ambiente conda (default: 'labauto')
    %
    % Compatibilità: Windows, macOS (parziale), Linux (parziale)
    
    % Argomento di default
    if nargin < 1
        envName = 'labauto';
    end
    
    % Determina il sistema operativo
    isWindows = ispc;
    isMac = ismac;
    isLinux = isunix && ~ismac;
    
    % Comando per trovare Python dipende dal sistema operativo
    if isWindows
        [~, python_locations] = system('where python');
    elseif isMac || isLinux
        [~, python_locations] = system('which python3');
    else
        error('Sistema operativo non supportato');
    end
    
    python_paths = splitlines(python_locations);
    flag = false;
    
    for ip = 1:length(python_paths)
        current_path = python_paths{ip};
        
        % Salta percorsi vuoti
        if isempty(current_path)
            continue
        end
        
        % Windows: esclude Microsoft Store Python
        if isWindows && contains(current_path, 'Microsoft')
            warning(current_path + " is not supported");
            continue
        end
        
        % Cerca anaconda/miniconda
        if contains(current_path, 'anaconda') || contains(current_path, 'miniconda')
            if ~contains(current_path, envName)
                % Costruisci il percorso dell'ambiente conda
                if isWindows
                    % Windows: C:\...\Anaconda3\envs\labauto
                    parts = split(current_path, filesep);
                    anaconda_root = fullfile(parts{1:end-1});
                    condaEnvRoot = fullfile(anaconda_root, 'envs', envName);
                    
                    % Percorsi da aggiungere al PATH
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
                    
                    % Aggiungi PATH corrente
                    currentPath = strsplit(getenv('PATH'), ';')';
                    newPath = [pathsToAdd; currentPath];
                    newPath = unique(newPath, 'stable');
                    setenv('PATH', strjoin(newPath, ';'));
                    
                    % Configura Python
                    pyExec = fullfile(condaEnvRoot, 'python.exe');
                    pyversion(pyExec);
                    pyenv('Version', pyExec);
                    
                elseif isMac || isLinux
                    % macOS/Linux: /Users/.../anaconda3/envs/labauto
                    % Su questi sistemi il PATH è separato da ':' non ';'
                    parts = strsplit(current_path, filesep);
                    anaconda_root = fullfile(parts{1:end-1});
                    condaEnvRoot = fullfile(anaconda_root, 'envs', envName);
                    
                    % Percorsi da aggiungere (versione ridotta per Unix-like)
                    pathsToAdd = {
                        fullfile(condaEnvRoot, 'bin');
                        fullfile(condaEnvRoot, 'lib');
                    };
                    
                    % Aggiungi PATH corrente
                    currentPath = strsplit(getenv('PATH'), ':')';
                    newPath = [pathsToAdd; currentPath];
                    newPath = unique(newPath, 'stable');
                    setenv('PATH', strjoin(newPath, ':'));
                    
                    % Configura Python
                    pyExec = fullfile(condaEnvRoot, 'bin', 'python');
                    pyversion(pyExec);
                    pyenv('Version', pyExec);
                end
                
                flag = true;
                fprintf('✓ Ambiente Python "%s" configurato correttamente\n', envName);
                break
            else
                % Se è già l'ambiente cercato, usalo direttamente
                pyversion(current_path);
                flag = true;
                fprintf('✓ Ambiente Python "%s" trovato\n', envName);
                break
            end
        end
    end
    
    if ~flag
        error('Impossibile trovare un ambiente Python valido con nome "%s"', envName);
    end
end