classdef fileSelector < handle
    % FILESELECTOR Classe per selezionare un file tramite GUI con radiobutton
    
    properties (Access = private)
        fig
        bg
        selected_file = []
        done = false
    end
    
    methods (Access = public)
        function selected = show(obj, file_names)
            % SHOW Mostra la GUI e ritorna il file selezionato
            %
            % Sintassi:
            %   selected = FileSelector.show(file_names)
            %
            % Input:
            %   file_names (cell array) - Lista dei nomi dei file
            %
            % Output:
            %   selected (char) - Nome del file selezionato, o [] se annullato
            
            %obj.done = false;
            obj.create_gui(file_names);
            
            % Aspetta finché non è fatto
            while ~obj.done
                pause(0.1);
                %drawnow;
            end
            
            selected = obj.selected_file;
        end
    end
    
    methods (Access = private)
        function create_gui(obj, file_names)
            % Crea la GUI
            obj.fig = uifigure('Name', 'Seleziona file chirp_experiment', ...
                               'NumberTitle', 'off', ...
                               'WindowStyle', 'modal', ...
                               'Position', [100, 100, 500, 80 + length(file_names) * 30], ...
                               'CloseRequestFcn', @(src, event) obj.on_close());
            
            % Aggiungi label
            uilabel(obj.fig, ...
                    'Text', 'Seleziona un file:', ...
                    'Position', [20, 50 + length(file_names) * 30, 460, 22], ...
                    'FontSize', 12, ...
                    'FontWeight', 'bold');
            
            % Crea un button group con radiobutton
            obj.bg = uibuttongroup(obj.fig, ...
                                   'Position', [20, 80, 460, length(file_names) * 30]);
            
            % Aggiungi radiobutton per ogni file
            for i = 1:length(file_names)
                uiradiobutton(obj.bg, ...
                              'Text', file_names{i}, ...
                              'Position', [20, length(file_names) * 30 - i * 30 + 10, 420, 22], ...
                              'FontSize', 10);
            end
            
            % Seleziona il primo file come default
            obj.bg.SelectedObject = obj.bg.Children(end);
            
            % Crea i pulsanti OK e Annulla
            uibutton(obj.fig, ...
                     'Text', 'OK', ...
                     'Position', [120, 20, 100, 30], ...
                     'ButtonPushedFcn', @(src, event) obj.ok_callback());
            
            uibutton(obj.fig, ...
                     'Text', 'Annulla', ...
                     'Position', [280, 20, 100, 30], ...
                     'ButtonPushedFcn', @(src, event) obj.cancel_callback());
        end
        
        function ok_callback(obj)
            % Callback del pulsante OK
            if ~isempty(obj.bg.SelectedObject)
                obj.selected_file = obj.bg.SelectedObject.Text;
                fprintf('File selezionato: %s\n', obj.selected_file);
            end
            obj.done = true;
            delete(obj.fig);
        end
        
        function cancel_callback(obj)
            % Callback del pulsante Annulla
            obj.selected_file = [];
            fprintf('Selezione annullata.\n');
            obj.done = true;
            %delete(obj.fig);
        end
        
        function on_close(obj)
            % Callback quando si chiude la finestra con la X
            obj.selected_file = [];
            obj.done = true;
            delete(obj.fig);
        end
    end
end