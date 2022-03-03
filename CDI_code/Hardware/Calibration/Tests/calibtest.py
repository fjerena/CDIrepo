from tkinter import *

root = Tk()
'''
Label(root, text="Nombre:").grid(pady=5, row=4, column=0)
Label(root, text="Apellido:").grid( pady=5, row=5, column=0)

Entry(root, width=40).grid(padx=5, row=4, column=1)
Entry(root, width=40).grid(padx=5, row=5, column=1)

Button(root, text="Enviar", width=50).grid(padx=10, pady=10, row=6, column=0, columnspan=2)

for r in range(0, 2):
    for c in range(0, 8):
        cell = Entry(root, width=10)
        cell.grid(padx=5, pady=5, row=r, column=c)
        cell.insert(0, '({}, {})'.format(r, c))
'''
root.mainloop()