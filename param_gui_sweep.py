import tkinter as tk
from tkinter import ttk, messagebox

def get_sweepable_params(params, path_prefix=()):
    paths = []
    for k, v in params.items():
        current_path = path_prefix + (k,)
        if isinstance(v, dict) and 'value' in v:
            paths.append(".".join(current_path))
        elif isinstance(v, dict):
            paths.extend(get_sweepable_params(v, current_path))
        elif isinstance(v, (int, float)) and not isinstance(v, bool):
            paths.append(".".join(current_path))
    return paths

def edit_params_for_sweep(params):
    """
    Opens a GUI to edit the parameters dictionary and pick sweeps.
    Returns (modified_params, sweep_config) or exits if cancelled.
    """
    root = tk.Tk()
    root.title("YMD Parameter & Sweep Editor")
    root.geometry("900x800")

    style = ttk.Style()
    style.theme_use('clam')

    main_frame = ttk.Frame(root, padding="10")
    main_frame.pack(fill=tk.BOTH, expand=True)

    notebook = ttk.Notebook(main_frame)
    notebook.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

    entries = {}

    def create_widgets(parent, data, path_prefix):
        row = 0
        for key, value in data.items():
            current_path = path_prefix + (key,)
            if isinstance(value, dict) and 'value' in value:
                label_text = key.replace('_', ' ').capitalize()
                ttk.Label(parent, text=label_text).grid(row=row, column=0, sticky=tk.W, padx=5, pady=2)
                
                entry = ttk.Entry(parent, width=15)
                entry.insert(0, str(value['value']))
                entry.grid(row=row, column=1, sticky=tk.W, padx=5, pady=2)
                entries[current_path + ('value',)] = entry

                if 'unit' in value:
                    ttk.Label(parent, text=value['unit']).grid(row=row, column=2, sticky=tk.W, padx=5, pady=2)
                elif 'units' in value:
                     ttk.Label(parent, text=value['units']).grid(row=row, column=2, sticky=tk.W, padx=5, pady=2)
                
                row += 1
            elif isinstance(value, dict):
                labelframe = ttk.LabelFrame(parent, text=key.replace('_', ' ').capitalize(), padding="5")
                labelframe.grid(row=row, column=0, columnspan=3, sticky=tk.EW, padx=5, pady=5)
                create_widgets(labelframe, value, current_path)
                row += 1
            else:
                label_text = key.replace('_', ' ').capitalize()
                ttk.Label(parent, text=label_text).grid(row=row, column=0, sticky=tk.W, padx=5, pady=2)
                
                entry = ttk.Entry(parent, width=15)
                entry.insert(0, str(value))
                entry.grid(row=row, column=1, sticky=tk.W, padx=5, pady=2)
                entries[current_path] = entry
                row += 1

    # Create tabs for base_params
    for key, value in params.items():
        if isinstance(value, dict):
            tab_frame = ttk.Frame(notebook, padding="10")
            notebook.add(tab_frame, text=key.capitalize())
            create_widgets(tab_frame, value, (key,))

    sweepable_params = get_sweepable_params(params)
    sweepable_params.insert(0, "velocity") # Ensure velocity can also be swept
    
    # KPIs available for plotting
    kpis = ["Grip Limit (Acceleration)", "Limit Balance (Yaw Moment)", "Control", "Stability"]

    # 1D Sweep Tab
    tab_1d = ttk.Frame(notebook, padding="10")
    notebook.add(tab_1d, text="1D Sweep")
    
    ttk.Label(tab_1d, text="Select Parameter to Sweep:").grid(row=0, column=0, sticky=tk.W, pady=5)
    param1_1d_var = tk.StringVar()
    if sweepable_params: param1_1d_var.set(sweepable_params[0])
    param1_1d_cb = ttk.Combobox(tab_1d, textvariable=param1_1d_var, values=sweepable_params, width=40)
    param1_1d_cb.grid(row=0, column=1, sticky=tk.W, pady=5, padx=5)
    
    ttk.Label(tab_1d, text="Start Value:").grid(row=1, column=0, sticky=tk.W, pady=5)
    start_1d_entry = ttk.Entry(tab_1d, width=15)
    start_1d_entry.insert(0, "10.0")
    start_1d_entry.grid(row=1, column=1, sticky=tk.W, pady=5, padx=5)

    ttk.Label(tab_1d, text="End Value:").grid(row=2, column=0, sticky=tk.W, pady=5)
    end_1d_entry = ttk.Entry(tab_1d, width=15)
    end_1d_entry.insert(0, "50.0")
    end_1d_entry.grid(row=2, column=1, sticky=tk.W, pady=5, padx=5)

    ttk.Label(tab_1d, text="Num Steps:").grid(row=3, column=0, sticky=tk.W, pady=5)
    steps_1d_entry = ttk.Entry(tab_1d, width=15)
    steps_1d_entry.insert(0, "5")
    steps_1d_entry.grid(row=3, column=1, sticky=tk.W, pady=5, padx=5)

    ttk.Label(tab_1d, text="KPIs to Plot:").grid(row=4, column=0, sticky=tk.NW, pady=5)
    kpi_1d_vars = {}
    kpi_frame_1d = ttk.Frame(tab_1d)
    kpi_frame_1d.grid(row=4, column=1, sticky=tk.W, pady=5, padx=5)
    for i, kpi in enumerate(kpis):
        var = tk.BooleanVar(value=True)
        ttk.Checkbutton(kpi_frame_1d, text=kpi, variable=var).grid(row=i, column=0, sticky=tk.W)
        kpi_1d_vars[kpi] = var

    # 2D Sweep Tab
    tab_2d = ttk.Frame(notebook, padding="10")
    notebook.add(tab_2d, text="2D Sweep")
    
    ttk.Label(tab_2d, text="Select Parameter 1:").grid(row=0, column=0, sticky=tk.W, pady=5)
    param1_2d_var = tk.StringVar()
    if sweepable_params: param1_2d_var.set(sweepable_params[0])
    param1_2d_cb = ttk.Combobox(tab_2d, textvariable=param1_2d_var, values=sweepable_params, width=40)
    param1_2d_cb.grid(row=0, column=1, sticky=tk.W, pady=5, padx=5)
    
    ttk.Label(tab_2d, text="Start 1:").grid(row=1, column=0, sticky=tk.W, pady=5)
    start1_2d_entry = ttk.Entry(tab_2d, width=15); start1_2d_entry.insert(0, "10.0")
    start1_2d_entry.grid(row=1, column=1, sticky=tk.W, pady=5, padx=5)

    ttk.Label(tab_2d, text="End 1:").grid(row=2, column=0, sticky=tk.W, pady=5)
    end1_2d_entry = ttk.Entry(tab_2d, width=15); end1_2d_entry.insert(0, "50.0")
    end1_2d_entry.grid(row=2, column=1, sticky=tk.W, pady=5, padx=5)

    ttk.Label(tab_2d, text="Steps 1:").grid(row=3, column=0, sticky=tk.W, pady=5)
    steps1_2d_entry = ttk.Entry(tab_2d, width=15); steps1_2d_entry.insert(0, "5")
    steps1_2d_entry.grid(row=3, column=1, sticky=tk.W, pady=5, padx=5)

    ttk.Separator(tab_2d, orient='horizontal').grid(row=4, column=0, columnspan=2, sticky=tk.EW, pady=10)

    ttk.Label(tab_2d, text="Select Parameter 2:").grid(row=5, column=0, sticky=tk.W, pady=5)
    param2_2d_var = tk.StringVar()
    if len(sweepable_params) > 1: param2_2d_var.set(sweepable_params[1])
    elif sweepable_params: param2_2d_var.set(sweepable_params[0])
    param2_2d_cb = ttk.Combobox(tab_2d, textvariable=param2_2d_var, values=sweepable_params, width=40)
    param2_2d_cb.grid(row=5, column=1, sticky=tk.W, pady=5, padx=5)

    ttk.Label(tab_2d, text="Start 2:").grid(row=6, column=0, sticky=tk.W, pady=5)
    start2_2d_entry = ttk.Entry(tab_2d, width=15); start2_2d_entry.insert(0, "40.0")
    start2_2d_entry.grid(row=6, column=1, sticky=tk.W, pady=5, padx=5)

    ttk.Label(tab_2d, text="End 2:").grid(row=7, column=0, sticky=tk.W, pady=5)
    end2_2d_entry = ttk.Entry(tab_2d, width=15); end2_2d_entry.insert(0, "60.0")
    end2_2d_entry.grid(row=7, column=1, sticky=tk.W, pady=5, padx=5)

    ttk.Label(tab_2d, text="Steps 2:").grid(row=8, column=0, sticky=tk.W, pady=5)
    steps2_2d_entry = ttk.Entry(tab_2d, width=15); steps2_2d_entry.insert(0, "5")
    steps2_2d_entry.grid(row=8, column=1, sticky=tk.W, pady=5, padx=5)

    ttk.Separator(tab_2d, orient='horizontal').grid(row=9, column=0, columnspan=2, sticky=tk.EW, pady=10)

    ttk.Label(tab_2d, text="KPIs to Plot:").grid(row=10, column=0, sticky=tk.NW, pady=5)
    kpi_2d_vars = {}
    kpi_frame_2d = ttk.Frame(tab_2d)
    kpi_frame_2d.grid(row=10, column=1, sticky=tk.W, pady=5, padx=5)
    for i, kpi in enumerate(kpis):
        var = tk.BooleanVar(value=True)
        ttk.Checkbutton(kpi_frame_2d, text=kpi, variable=var).grid(row=i, column=0, sticky=tk.W)
        kpi_2d_vars[kpi] = var


    button_frame = ttk.Frame(main_frame)
    button_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=10)

    result_params = [None]
    sweep_config = {'type': 'none'}

    def update_base_dict():
        import copy
        new_params = copy.deepcopy(params)
        for path, entry in entries.items():
            val = entry.get()
            try:
                if '.' in val: val = float(val)
                else: val = int(val)
            except ValueError:
                pass
            
            current = new_params
            for k in path[:-1]:
                current = current[k]
            current[path[-1]] = val
        return new_params

    def on_run_base():
        try:
            result_params[0] = update_base_dict()
            sweep_config['type'] = 'none'
            root.destroy()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to parse parameters: {e}")

    def on_run_1d():
        try:
            result_params[0] = update_base_dict()
            sweep_config['type'] = '1d'
            sweep_config['param1'] = param1_1d_var.get()
            sweep_config['start1'] = float(start_1d_entry.get())
            sweep_config['end1'] = float(end_1d_entry.get())
            sweep_config['steps1'] = int(steps_1d_entry.get())
            sweep_config['kpis'] = [k for k, v in kpi_1d_vars.items() if v.get()]
            root.destroy()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to parse 1D sweep param: {e}")

    def on_run_2d():
        try:
            result_params[0] = update_base_dict()
            sweep_config['type'] = '2d'
            sweep_config['param1'] = param1_2d_var.get()
            sweep_config['start1'] = float(start1_2d_entry.get())
            sweep_config['end1'] = float(end1_2d_entry.get())
            sweep_config['steps1'] = int(steps1_2d_entry.get())

            sweep_config['param2'] = param2_2d_var.get()
            sweep_config['start2'] = float(start2_2d_entry.get())
            sweep_config['end2'] = float(end2_2d_entry.get())
            sweep_config['steps2'] = int(steps2_2d_entry.get())

            sweep_config['kpis'] = [k for k, v in kpi_2d_vars.items() if v.get()]
            root.destroy()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to parse 2D sweep param: {e}")

    def on_cancel():
        root.destroy()
        exit()

    ttk.Button(button_frame, text="Cancel", command=on_cancel).pack(side=tk.RIGHT, padx=5)
    ttk.Button(button_frame, text="Run 2D Sweep", command=on_run_2d).pack(side=tk.RIGHT, padx=5)
    ttk.Button(button_frame, text="Run 1D Sweep", command=on_run_1d).pack(side=tk.RIGHT, padx=5)
    ttk.Button(button_frame, text="Run Base YMD", command=on_run_base).pack(side=tk.RIGHT, padx=5)

    root.update_idletasks()
    width = root.winfo_width()
    height = root.winfo_height()
    x = (root.winfo_screenwidth() // 2) - (width // 2)
    y = (root.winfo_screenheight() // 2) - (height // 2)
    root.geometry('{}x{}+{}+{}'.format(width, height, x, y))

    root.protocol("WM_DELETE_WINDOW", on_cancel)

    root.mainloop()
    
    return result_params[0], sweep_config

if __name__ == "__main__":
    import yaml
    with open('base_params.yaml', 'r') as f:
        p = yaml.safe_load(f)
    b, s = edit_params_for_sweep(p)
    if b:
        print("Base params updated.")
        print("Sweep config:", s)
