#!/usr/bin/env python3
# Quita los comentarios (#) y los textos de documentacion (docstrings) de archivos Python sin
# cambiar lo que hace el programa: al final compara el arbol de sintaxis (AST) con el original.
#   python quitar_comentarios.py <carpeta_origen> <carpeta_destino>
import ast
import io
import os
import sys
import tokenize


def rangos_docstrings(arbol):
    """(linea_inicio, linea_fin) de cada docstring de modulo, clase o funcion."""
    out = []
    for nodo in ast.walk(arbol):
        if isinstance(nodo, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            cuerpo = nodo.body
            if cuerpo and isinstance(cuerpo[0], ast.Expr) and isinstance(getattr(cuerpo[0], "value", None), ast.Constant) \
                    and isinstance(cuerpo[0].value.value, str):
                out.append((cuerpo[0].lineno, cuerpo[0].end_lineno, nodo))
    return out


def sin_docstrings(arbol):
    for nodo in ast.walk(arbol):
        if isinstance(nodo, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            c = nodo.body
            if c and isinstance(c[0], ast.Expr) and isinstance(getattr(c[0], "value", None), ast.Constant) \
                    and isinstance(c[0].value.value, str):
                nodo.body = c[1:] or [ast.Pass()]
    return ast.dump(arbol)


def limpiar(texto):
    arbol = ast.parse(texto)
    lineas = texto.splitlines(keepends=True)
    borrar = set()
    reemplazo = {}
    for ini, fin, nodo in rangos_docstrings(arbol):
        solo_doc = len(nodo.body) == 1 and not isinstance(nodo, ast.Module)
        if solo_doc:                     # una funcion que solo tenia docstring necesita un cuerpo
            sangria = lineas[ini - 1][:len(lineas[ini - 1]) - len(lineas[ini - 1].lstrip())]
            reemplazo[ini] = sangria + "pass\n"
            borrar.update(range(ini + 1, fin + 1))
        else:
            borrar.update(range(ini, fin + 1))
    # comentarios: se corta la linea donde empieza el token COMMENT
    cortes = {}
    for tok in tokenize.generate_tokens(io.StringIO(texto).readline):
        if tok.type == tokenize.COMMENT:
            fila, col = tok.start
            cortes[fila] = min(col, cortes.get(fila, col))
    out = []
    for i, ln in enumerate(lineas, 1):
        if i in reemplazo:
            out.append(reemplazo[i])
            continue
        if i in borrar:
            continue
        if i in cortes:
            resto = ln[:cortes[i]].rstrip()
            if not resto.strip():
                continue                 # la linea era solo un comentario
            ln = resto + "\n"
        out.append(ln)
    # no dejar mas de una linea en blanco seguida (ni al principio)
    limpio, vacias = [], 0
    for ln in out:
        if ln.strip():
            vacias = 0
            limpio.append(ln)
        else:
            vacias += 1
            if vacias <= 1 and limpio:
                limpio.append("\n")
    nuevo = "".join(limpio).rstrip() + "\n"
    if sin_docstrings(ast.parse(texto)) != sin_docstrings(ast.parse(nuevo)):
        raise RuntimeError("el programa cambio al quitar los comentarios")
    return nuevo


def main(origen, destino):
    os.makedirs(destino, exist_ok=True)
    for nombre in sorted(os.listdir(origen)):
        p = os.path.join(origen, nombre)
        if not os.path.isfile(p):
            continue
        datos = open(p, encoding="utf-8").read()
        if nombre.endswith(".py"):
            nuevo = limpiar(datos)
            print("%-16s %5d -> %5d lineas" % (nombre, datos.count("\n"), nuevo.count("\n")))
        else:
            nuevo = datos
        open(os.path.join(destino, nombre), "w", encoding="utf-8", newline="\n").write(nuevo)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
