# Autoría del software

Yo, **Alejandra Obando Cortés** (estudiante de Ingeniería Mecatrónica, Universidad Autónoma de Occidente), desarrollé
el software de este repositorio: la arquitectura en ROS 2, los drivers, el freno con LiDAR, la navegación, la máquina
de estados de la misión, la visión de frutas y cajas, el reconocimiento de voz, el control del brazo y la red de
aprendizaje por refuerzo.

## Por qué los commits aparecen como "sebas graja"

Trabajé desde el **computador de mi casa**, que tenía git configurado con el nombre y el correo de Sebastián Grajales
(`sebas graja <sebasgrajalesp28@gmail.com>`). Por eso los commits hechos ahí quedaron firmados con ese nombre, y por
eso el repositorio está en esa cuenta de GitHub. Mi portátil es pesado para llevarlo a casa, así que ese era el
computador en el que programaba.

- **74 commits** firmados como `sebas graja`, entre el 27/10/2026 y el 4/11/2026: hechos por mí desde ese computador.
- **Commits del 9/11/2026** en adelante: hechos desde mi propio computador, con mi cuenta `alejandra-obando`.

El historial **no se reescribió**: cada commit conserva su firma y su hash originales. El archivo
[`.mailmap`](.mailmap) usa el mecanismo estándar de git para atribuir esos commits a mi nombre. Para verlo:

```bash
git shortlog -sn              # con .mailmap: todos los commits a nombre de Alejandra Obando Cortes
git log --format='%h %aN'     # autor según .mailmap
git log --format='%h %an'     # firma original de cada commit
```

La gráfica de contribuidores de la página de GitHub no lee `.mailmap`, por eso ahí sigue apareciendo la firma original.

## Código evaluado

El código presentado en la evaluación del 4/11/2026 es la etiqueta
[`evaluacion-4nov`](https://github.com/sebas2868/Wallens_neuro/tree/evaluacion-4nov). La evidencia de que es el mismo
que corrió en el robot está en [`docs/evidencia-evaluacion-4nov/`](docs/evidencia-evaluacion-4nov/README.md).

Alejandra Obando Cortés, 9 de noviembre de 2026.
