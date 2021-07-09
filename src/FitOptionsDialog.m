classdef FitOptionsDialog < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                   matlab.ui.Figure
        MethodLabel                matlab.ui.control.Label
        MethodDropDown             matlab.ui.control.DropDown
        RobustDropDownLabel        matlab.ui.control.Label
        RobustDropDown             matlab.ui.control.DropDown
        AlgorithmLabel             matlab.ui.control.Label
        AlgorithmDropDown          matlab.ui.control.DropDown
        DiffMinChangeLabel         matlab.ui.control.Label
        DiffMinChangeEditField     matlab.ui.control.NumericEditField
        DiffMaxChangeLabel         matlab.ui.control.Label
        DiffMaxChangeEditField     matlab.ui.control.NumericEditField
        MaxFunEvalsEditFieldLabel  matlab.ui.control.Label
        MaxFunEvalsEditField       matlab.ui.control.NumericEditField
        TolXEditFieldLabel         matlab.ui.control.Label
        TolXEditField              matlab.ui.control.NumericEditField
        TolFunEditFieldLabel       matlab.ui.control.Label
        TolFunEditField            matlab.ui.control.NumericEditField
        MaxIterEditFieldLabel      matlab.ui.control.Label
        MaxIterEditField           matlab.ui.control.NumericEditField
        CoeffTable                    matlab.ui.control.Table
        Cancel                     matlab.ui.control.Button
        ApplyButton                matlab.ui.control.Button
        NormalizeDropDownLabel     matlab.ui.control.Label
        NormalizeDropDown          matlab.ui.control.DropDown
    end

    
    properties (Access = private)
        CallingApp % Description
        fo;
        CoeffData;
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app, mainapp, callingFO, coeffs)
            app.CallingApp = mainapp;
            app.fo = callingFO;
            app.NormalizeDropDown.Value      = regexprep(app.fo.Normalize,'(\<\w)','${upper($1)}'); %Capitalize first letter
            app.RobustDropDown.Value         = app.fo.Robust        ;
            app.AlgorithmDropDown.Value      = app.fo.Algorithm     ;
            app.DiffMinChangeEditField.Value = app.fo.DiffMinChange ;
            app.DiffMaxChangeEditField.Value = app.fo.DiffMaxChange ;
            app.MaxFunEvalsEditField.Value   = app.fo.MaxFunEvals   ;
            app.MaxIterEditField.Value       = app.fo.MaxIter       ;
            app.TolFunEditField.Value        = app.fo.TolFun        ;
            app.TolXEditField.Value          = app.fo.TolX          ;
            for k = 1:length(coeffs)
                nextRow = {coeffs{k}, app.fo.StartPoint(k), app.fo.Lower(k), app.fo.Upper(k)};
                app.CoeffTable.Data = [app.CoeffTable.Data; nextRow]; 
            end
        end

        % Button pushed function: ApplyButton
        function ApplyButtonPushed(app, event)
            % app.fo.Method = app.MethodDropDown.Value;
            app.fo.Normalize = app.NormalizeDropDown.Value;
            app.fo.Robust = app.RobustDropDown.Value;
            app.fo.Algorithm = app.AlgorithmDropDown.Value;
            app.fo.DiffMinChange = app.DiffMinChangeEditField.Value;
            app.fo.DiffMaxChange = app.DiffMaxChangeEditField.Value;
            app.fo.MaxFunEvals = app.MaxFunEvalsEditField.Value;
            app.fo.MaxIter = app.MaxIterEditField.Value;
            app.fo.TolFun = app.TolFunEditField.Value;
            app.fo.TolX = app.TolXEditField.Value;
            app.fo.StartPoint = [app.CoeffTable.Data{:, 2}];
            app.fo.Lower = [app.CoeffTable.Data{:,3}];
            app.fo.Upper = [app.CoeffTable.Data{:,4}];
            app.CallingApp.updateFitOptions(app.fo);
            delete(app);
        end

        % Button pushed function: Cancel
        function CancelPushed(app, event)
            delete(app);
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 326 506];
            app.UIFigure.Name = 'MATLAB App';

            % Create MethodLabel
            app.MethodLabel = uilabel(app.UIFigure);
            app.MethodLabel.HorizontalAlignment = 'right';
            app.MethodLabel.Position = [4 478 49 22];
            app.MethodLabel.Text = 'Method:';

            % Create MethodDropDown
            app.MethodDropDown = uidropdown(app.UIFigure);
            app.MethodDropDown.Items = {'NonlinearLeastSquares', 'LinearLeastSquares'};
            app.MethodDropDown.Position = [103 478 212 22];
            app.MethodDropDown.Value = 'NonlinearLeastSquares';

            % Create RobustDropDownLabel
            app.RobustDropDownLabel = uilabel(app.UIFigure);
            app.RobustDropDownLabel.HorizontalAlignment = 'right';
            app.RobustDropDownLabel.Position = [4 418 47 22];
            app.RobustDropDownLabel.Text = 'Robust:';

            % Create RobustDropDown
            app.RobustDropDown = uidropdown(app.UIFigure);
            app.RobustDropDown.Items = {'Off', 'LAR', 'Bisquare'};
            app.RobustDropDown.Position = [103 418 212 22];
            app.RobustDropDown.Value = 'Off';

            % Create AlgorithmLabel
            app.AlgorithmLabel = uilabel(app.UIFigure);
            app.AlgorithmLabel.HorizontalAlignment = 'right';
            app.AlgorithmLabel.Position = [4 388 59 22];
            app.AlgorithmLabel.Text = 'Algorithm:';

            % Create AlgorithmDropDown
            app.AlgorithmDropDown = uidropdown(app.UIFigure);
            app.AlgorithmDropDown.Items = {'Trust-Region', 'Levenberg-Marquardt'};
            app.AlgorithmDropDown.Position = [103 388 212 22];
            app.AlgorithmDropDown.Value = 'Trust-Region';

            % Create DiffMinChangeLabel
            app.DiffMinChangeLabel = uilabel(app.UIFigure);
            app.DiffMinChangeLabel.HorizontalAlignment = 'right';
            app.DiffMinChangeLabel.Position = [4 358 88 22];
            app.DiffMinChangeLabel.Text = 'DiffMinChange:';

            % Create DiffMinChangeEditField
            app.DiffMinChangeEditField = uieditfield(app.UIFigure, 'numeric');
            app.DiffMinChangeEditField.Position = [103 358 212 22];
            app.DiffMinChangeEditField.Value = 1e-08;

            % Create DiffMaxChangeLabel
            app.DiffMaxChangeLabel = uilabel(app.UIFigure);
            app.DiffMaxChangeLabel.HorizontalAlignment = 'right';
            app.DiffMaxChangeLabel.Position = [4 329 91 22];
            app.DiffMaxChangeLabel.Text = 'DiffMaxChange:';

            % Create DiffMaxChangeEditField
            app.DiffMaxChangeEditField = uieditfield(app.UIFigure, 'numeric');
            app.DiffMaxChangeEditField.Position = [103 329 212 22];
            app.DiffMaxChangeEditField.Value = 0.1;

            % Create MaxFunEvalsEditFieldLabel
            app.MaxFunEvalsEditFieldLabel = uilabel(app.UIFigure);
            app.MaxFunEvalsEditFieldLabel.HorizontalAlignment = 'right';
            app.MaxFunEvalsEditFieldLabel.Position = [4 300 82 22];
            app.MaxFunEvalsEditFieldLabel.Text = 'MaxFunEvals:';

            % Create MaxFunEvalsEditField
            app.MaxFunEvalsEditField = uieditfield(app.UIFigure, 'numeric');
            app.MaxFunEvalsEditField.Position = [103 300 212 22];
            app.MaxFunEvalsEditField.Value = 600;

            % Create TolXEditFieldLabel
            app.TolXEditFieldLabel = uilabel(app.UIFigure);
            app.TolXEditFieldLabel.HorizontalAlignment = 'right';
            app.TolXEditFieldLabel.Position = [4 213 32 22];
            app.TolXEditFieldLabel.Text = 'TolX:';

            % Create TolXEditField
            app.TolXEditField = uieditfield(app.UIFigure, 'numeric');
            app.TolXEditField.Position = [103 213 212 22];
            app.TolXEditField.Value = 1e-06;

            % Create TolFunEditFieldLabel
            app.TolFunEditFieldLabel = uilabel(app.UIFigure);
            app.TolFunEditFieldLabel.HorizontalAlignment = 'right';
            app.TolFunEditFieldLabel.Position = [4 242 45 22];
            app.TolFunEditFieldLabel.Text = 'TolFun:';

            % Create TolFunEditField
            app.TolFunEditField = uieditfield(app.UIFigure, 'numeric');
            app.TolFunEditField.Position = [103 242 212 22];
            app.TolFunEditField.Value = 1e-06;

            % Create MaxIterEditFieldLabel
            app.MaxIterEditFieldLabel = uilabel(app.UIFigure);
            app.MaxIterEditFieldLabel.HorizontalAlignment = 'right';
            app.MaxIterEditFieldLabel.Position = [4 271 49 22];
            app.MaxIterEditFieldLabel.Text = 'MaxIter:';

            % Create MaxIterEditField
            app.MaxIterEditField = uieditfield(app.UIFigure, 'numeric');
            app.MaxIterEditField.Position = [103 271 212 22];
            app.MaxIterEditField.Value = 400;

            % Create CoeffTable
            app.CoeffTable = uitable(app.UIFigure);
            app.CoeffTable.ColumnName = {'Coeff'; 'Start'; 'Lower'; 'Upper'};
            app.CoeffTable.RowName = {};
            app.CoeffTable.ColumnEditable = [true true true true true true];
            app.CoeffTable.Position = [11 43 304 159];

            % Create Cancel
            app.Cancel = uibutton(app.UIFigure, 'push');
            app.Cancel.ButtonPushedFcn = createCallbackFcn(app, @CancelPushed, true);
            app.Cancel.Position = [11 9 142 26];
            app.Cancel.Text = 'Cancel';

            % Create ApplyButton
            app.ApplyButton = uibutton(app.UIFigure, 'push');
            app.ApplyButton.ButtonPushedFcn = createCallbackFcn(app, @ApplyButtonPushed, true);
            app.ApplyButton.Position = [173 9 142 26];
            app.ApplyButton.Text = 'Apply';

            % Create NormalizeDropDownLabel
            app.NormalizeDropDownLabel = uilabel(app.UIFigure);
            app.NormalizeDropDownLabel.HorizontalAlignment = 'right';
            app.NormalizeDropDownLabel.Position = [4 448 63 22];
            app.NormalizeDropDownLabel.Text = 'Normalize:';

            % Create NormalizeDropDown
            app.NormalizeDropDown = uidropdown(app.UIFigure);
            app.NormalizeDropDown.Items = {'Off', 'On'};
            app.NormalizeDropDown.Position = [103 448 212 22];
            app.NormalizeDropDown.Value = 'Off';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = FitOptionsDialog(varargin)

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @(app)startupFcn(app, varargin{:}))

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end