# Colocación apoyada antes de reproducir

Fecha: 2026-11-06.

La pose separada del suelo es únicamente una semilla interna. `Simulation.place` deja caer el vehículo con controles neutros mediante el mismo motor, materiales, clasificación y parche físico que la simulación. No exige equilibrio, velocidades pequeñas ni cargas equilibradas. Termina cerca de la primera compresión máxima o después de 0,35 s de contacto si sigue descendiendo por una pendiente, con un máximo de 2 s físicos de preparación. Devuelve esa pose de contacto para la vista previa y el instante cero, aunque sea inestable.

La preparación no añade fotogramas, índices ni registros CSV a la simulación. Si falta cobertura dinámica o se agota el cálculo, usa la última pose con contacto; si todavía no la hay, elimina la separación de 20 cm de la semilla geométrica sobre los puntos observados. No rechaza una colocación por falta de estabilidad. Cambiar los materiales invalida la firma del apoyo; antes de simular se recalcula. La firma incluye la versión del criterio para actualizar colocaciones antiguas. Las velocidades iniciales elegidas por el usuario se mantienen para la simulación y no se usan como controles durante la preparación.

Los objetivos de ruta usan `purpose: waypoint` y conservan el criterio geométrico previo: no se exige que el vehículo pueda quedarse estacionado en cada punto de paso. Los parches de terreno se anclan a una cuadrícula global común de 20 cm, para que un desplazamiento durante el asentamiento no cambie la superficie muestreada al reiniciar.

## Verificación

- Suite completa actual: 56 pruebas Python y 16 de interfaz superadas. Sintaxis Python y JavaScript correcta.
- Siete pruebas físicas de colocación: los tres MAN, materiales activos, coordenadas topográficas elevadas, contactos ligeros, pendiente suave, escalón, pendientes inestables, destino de ruta sin estacionamiento y alternativa geométrica ante falta de cobertura dinámica. Todas superadas.
- Se comprueban contacto con el suelo y continuidad de posición/orientación en el primer fotograma. No se exige estabilidad posterior: la animación puede mostrar deslizamiento o vuelco.
- Una prueba nueva de interfaz comprueba materiales enviados a colocación y recálculo antes de simular tras cambiarlos. Las 16 pruebas de interfaz pasan.
- No se incorporaron dependencias ni se cambió la ley de fuerzas. La animación conserva la evolución física posterior al contacto.
