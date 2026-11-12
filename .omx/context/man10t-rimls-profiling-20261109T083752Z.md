# Profiling MAN10T en rimls.ply

- Petición: diez recorridos de diez segundos, tiempos reales, potencia, funciones costosas, informe PDF y archivo reproducible para comparar tras optimizar.
- Hipótesis de intención: establecer una línea base experimental comparable del simulador FusionForce.
- Evidencia: rimls.ply existe (4.401.684 bytes). DESIGN.md documenta el flujo del visor. El catálogo y el servidor implementan MAN_10t. El antecedente viewer/.cache/profiling/20261106-fresh/INFORME.md empleó otra nube, diez segundos simulados y perfíl Python tanto ligero como completo. El equipo muestra Intel Core Ultra 7 155H y potencia NVIDIA accesible; telemetría de CPU aún por comprobar.
- Por resolver: población de trayectorias, alcance del tiempo, régimen de caché y contacto, frontera eléctrica y alternativa sin sensor, criterios de validez y decisiones delegadas.
- Puntos técnicos: viewer/server.py, viewer/cloud_store.py, viewer/stability.py, viewer/catalog.json y el perfil anterior.
- Documentos revisados: instrucciones AGENTS de la sesión, DESIGN.md, informe y lanzador previos, inventario de .omx/context y .omx/specs. No se encontró README o glosario aplicable.
- Terminología: duración física de simulación, tiempo real de ejecución y modo de llegada a destino son magnitudes y comportamientos distintos.
- Resumen seguro del contexto inicial: not_needed.

- Ronda 1 [from-user]: diez trayectorias distintas de diez segundos simulados. Ronda 2 pendiente: alcance del tiempo y comparación.

- Ronda 2 [from-user]: referencia principal = tiempo del motor, con mapa ya cargado. La entrega al navegador queda fuera de esa métrica.
- La RTX devuelve vatios; la enumeración de contadores de Windows no mostró potencia de CPU. Alternativa sin vatios de CPU pendiente de decisión.
