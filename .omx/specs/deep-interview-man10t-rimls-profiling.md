# Especificación · Línea base MAN10T sobre rimls.ply

Perfil estándar, tres rondas, ambigüedad estimada del 11 % (umbral 20 %). Proyecto existente. Evidencia inicial en el contexto man10t-rimls-profiling de .omx/context.

- Intención y resultado: medir una línea base reproducible antes de optimizar el motor. Entregar PDF con gráficos y un paquete ejecutable con rutas, parámetros y datos brutos.
- Alcance: diez trayectorias distintas de diez segundos simulados. Tiempo real de evaluación del motor con el mapa cargado. Medir cada trayectoria sin cProfile y repetirla aparte para atribuir tiempo propio y acumulado.
- Exclusiones: carga del mapa, colocación previa, transporte/renderizado web y cambios de optimización en el código de producto.
- Decisiones: el agente selecciona y congela rutas y protocolo. Usar paso de 0,01 s, contacto completo y CSV desactivado para aislar el cálculo del registro en disco. Documentar cobertura, repeticiones diagnósticas, configuración y límites.
- Potencia: si falta telemetría fiable de CPU, entregar tiempo, carga de CPU y vatios de GPU, etiquetando el dominio eléctrico y dejando la potencia de CPU sin medir.
- Aceptación: diez resultados válidos a t=10,00 s, datos por ruta, estadística agregada, perfiles reproducibles con llamadas y explicación por función y PDF visualmente comprobado.
- Presión aplicada: ante una mejora del motor que no acelere el navegador, el usuario eligió medir el motor.
- Claridad estimada (intención, resultado, alcance, restricciones, éxito, contexto): 0,97 / 0,97 / 0,91 / 0,79 / 0,81 / 0,77.
