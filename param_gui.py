
import tkinter as tk
from tkinter import ttk, messagebox

def edit_params(params):
    """
    Opens a GUI to edit the parameters dictionary.
    Returns the modified dictionary if 'Run' is clicked, or None if cancelled.
    """
    root = tk.Tk()
    root.title("YMD Parameter Editor")
    root.geometry("600x600")

    # Style
    style = ttk.Style()
    style.theme_use('clam')

    # Main frame
    main_frame = ttk.Frame(root, padding="10")
    main_frame.pack(fill=tk.BOTH, expand=True)

    # Notebook for tabs
    notebook = ttk.Notebook(main_frame)
    notebook.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

    # Dictionary to store entry widgets and their corresponding param keys
    # Key: (tab_name, *nested_keys), Value: entry_widget
    entries = {}

    def create_widgets(parent, data, path_prefix):
        """
        Recursively create widgets for the dictionary `data`.
        `parent`: The parent widget (Frame).
        `path_prefix`: Tuple of keys to reach this level in the original dictionary.
        """
        row = 0
        for key, value in data.items():
            current_path = path_prefix + (key,)
            
            # Check if this is a "leaf" node with value/unit structure
            if isinstance(value, dict) and 'value' in value:
                # Label
                label_text = key.replace('_', ' ').capitalize()
                ttk.Label(parent, text=label_text).grid(row=row, column=0, sticky=tk.W, padx=5, pady=2)
                
                # Entry
                entry = ttk.Entry(parent, width=15)
                entry.insert(0, str(value['value']))
                entry.grid(row=row, column=1, sticky=tk.W, padx=5, pady=2)
                
                # Store reference to entry for later retrieval
                # We store the path to the 'value' key specifically
                entries[current_path + ('value',)] = entry

                # Unit Label (if exists)
                if 'unit' in value:
                    ttk.Label(parent, text=value['unit']).grid(row=row, column=2, sticky=tk.W, padx=5, pady=2)
                elif 'units' in value: # Handle 'units' typo in yaml
                     ttk.Label(parent, text=value['units']).grid(row=row, column=2, sticky=tk.W, padx=5, pady=2)
                
                row += 1
            
            # Recurse if it's a dictionary but NOT a value/unit leaf
            elif isinstance(value, dict):
                # Create a LabelFrame for nested dictionaries
                labelframe = ttk.LabelFrame(parent, text=key.replace('_', ' ').capitalize(), padding="5")
                labelframe.grid(row=row, column=0, columnspan=3, sticky=tk.EW, padx=5, pady=5)
                create_widgets(labelframe, value, current_path)
                row += 1
                
            # Direct value (leaf)
            else:
                label_text = key.replace('_', ' ').capitalize()
                ttk.Label(parent, text=label_text).grid(row=row, column=0, sticky=tk.W, padx=5, pady=2)
                
                entry = ttk.Entry(parent, width=15)
                entry.insert(0, str(value))
                entry.grid(row=row, column=1, sticky=tk.W, padx=5, pady=2)
                entries[current_path] = entry
                row += 1


    # Create tabs for top-level keys
    for key, value in params.items():
        if isinstance(value, dict):
            tab_frame = ttk.Frame(notebook, padding="10")
            notebook.add(tab_frame, text=key.capitalize())
            
            # Use a canvas for scrolling if needed, but for now just a frame inside
            # If params are many, scrolling might be needed. Let's start simple.
            
            # Use a scrollable frame implementation if needed, stick to simple for now
            # as requested "clean GUI"
            
            create_widgets(tab_frame, value, (key,))
        else:
            # For top-level non-dict items, put them in a "General" tab
            # But currently base_params.yaml has all dicts at top level except maybe if user changed it
            pass

    # Buttons
    button_frame = ttk.Frame(main_frame)
    button_frame.pack(fill=tk.X, pady=10)

    result_params = [None] # Use list to start mutable result

    def on_run():
        new_params = params.copy() # Shallow copy might not be enough for nested, but we are updating values in place
        # Actually, for deep updates, we need to be careful.
        # But here we are just updating the values at the paths.
        
        try:
            for path, entry in entries.items():
                val = entry.get()
                
                # Attempt to convert to number if possible
                try:
                    if '.' in val:
                        val = float(val)
                    else:
                        val = int(val)
                except ValueError:
                    pass # Keep as string
                
                # Update the nested dictionary
                d = new_params
                for k in path[:-1]:
                    d = d[k]
                d[path[-1]] = val
            
            result_params[0] = new_params
            root.destroy()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to parse parameters: {e}")

    def on_cancel():
        root.destroy()
        exit() # Terminate script immediately

    ttk.Button(button_frame, text="Run Analysis", command=on_run).pack(side=tk.RIGHT, padx=5)
    ttk.Button(button_frame, text="Cancel", command=on_cancel).pack(side=tk.RIGHT, padx=5)

    # Center window
    root.update_idletasks()
    width = root.winfo_width()
    height = root.winfo_height()
    x = (root.winfo_screenwidth() // 2) - (width // 2)
    y = (root.winfo_screenheight() // 2) - (height // 2)
    root.geometry('{}x{}+{}+{}'.format(width, height, x, y))

    root.protocol("WM_DELETE_WINDOW", on_cancel)

    root.mainloop()
    
    return result_params[0]

if __name__ == "__main__":
    # Test with dummy data
    import yaml
    with open('base_params.yaml', 'r') as f:
        p = yaml.safe_load(f)
    print(edit_params(p))
