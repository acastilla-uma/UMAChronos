# Verificación del panel de estabilidad

Fecha: 2026-11-06.

Implementado: LTR por cargas verticales de ruedas; PLTR cinemático inspirado en Li, identificado como adaptación; ZML/COP como coordenadas de una línea, sin escalar de seguridad inventado; SI energético con masa, geometría e inercia del vehículo activo. Fórmulas, hipótesis y parámetros visibles en el panel superior derecho.

Las curvas usan exclusivamente el prefijo de la reproducción hasta el instante seleccionado. Los índices se conservan al exportar e importar proyectos. Los datos ausentes se muestran como no disponibles.

## Evidencia

- `python -m unittest discover -s viewer/tests`: 49 pruebas superadas, incluida comparación con un monitor vacío para comprobar que la dinámica no cambia.
- `node --test viewer/tests/interface.test.mjs`: 15 pruebas superadas, incluida sincronización con pausa, avance, retroceso, reinicio y reapertura.
- `node --check viewer/app.js`: correcto.
- Revisión independiente de matemáticas: aprobada sin hallazgos accionables.
- Revisión independiente de interfaz: sincronización, fórmulas y unidades correctas; detectó recorte en ventanas de poca altura. Se amplió a 700 px el umbral que permite desplazar el panel completo y se repitieron las 15 pruebas de interfaz.

## Limitación de verificación

No se obtuvo una comprobación visual en navegador: el navegador integrado agotó el tiempo de acceso al servidor local. La disposición se revisó estáticamente. No se afirma validación visual ni validación experimental de los índices.
