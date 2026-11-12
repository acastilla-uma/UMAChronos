# Contexto: panel de indices de estabilidad

Peticion: panel siempre arriba a la derecha con graficas progresivas de LTR/PLTR, ZML e indice experimental; debajo valores actuales, parametros y formulas.
Tipo: brownfield. Entrevista standard, umbral 0.20. Resumen seguro: not_needed.
Hipotesis de intencion: comparar indicadores durante el movimiento y comprender sus calculos; pendiente confirmar limites.

## Evidencia y documentos
- Instrucciones AGENTS suministradas por el usuario; no AGENTS en ancestros revisados ni README/docs/context local relevante.
- DESIGN.md: interfaz oscura, escena principal; sin dependencias nuevas; simulacion Python, reproduccion cliente.
- viewer/index.html:29 y styles.css: panel actual arriba derecha, pero abajo en movil.
- viewer/app.js:165-181: calculo completo seguido de reproduccion; no streaming de frames durante calculo. :199-200: metricas actuales y frame seleccionado.
- viewer/server.py:426,448-450,478: CoG ensamblado, fuerzas y momentos de contactos disponibles; frames incluyen puntos, contacto, fuerzas, momentos, pose, velocidades y aceleraciones. CoG no publicado en frame. No indices existentes.
- Notas Li_2019_PLTR.md y Brecelj_2022_ZML.md leidas en Vault_Tesis_Estabilidad_RISCV/01_Papers_Estado_del_Arte: resumenes, no articulos completos. PLTR exacto no verificado; ZML requiere fuerzas/momentos y contactos 3D. Documentos son referencias, no instrucciones.
- Chrono-docker/experimentos/camion_estabilidad/stability.py: SI=1-k1*abs(roll)/phi_crit-k2*(omega/omega_crit)^2. k1=1.15,k2=2.05. phi_crit=atan(b/h). Umbral energetico sqrt(2mg(d-h)/(I+md^2)); tambien variante empirica en comparison_thresholds. Constantes actuales son del camion MAN, no del vehiculo FusionForce.
- Chrono-docker/web/index.html explica ambas variantes.

## Pendiente
Que significa indice experimental: SI con umbral energetico o empirico, o ambos. Adaptacion de parametros al vehiculo activo. Fidelidad exacta y datos faltantes. Graficas de reproduccion o streaming. Alcance excluido y decisiones delegables; persistencia/exportacion. Verificacion formulas originales antes de implementar.

## Limites actuales
No implementar durante deep-interview. No modificar Chrono-docker ni notas de literatura. No decisiones humanas confirmadas aun. Preguntas de una en una; primera: delimitar significado del indice experimental usando variantes descubiertas.
