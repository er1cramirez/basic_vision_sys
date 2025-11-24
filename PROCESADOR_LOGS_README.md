# Procesador de Logs de Simulación

## Descripción General

El script `process_simulation_logs.py` procesa automáticamente los logs CSV generados por las simulaciones UAV, creando:
- **Gráficas interactivas** individuales para cada archivo
- **Archivos de texto** formateados para parámetros de configuración
- **Gráficas de comparación** entre múltiples archivos
- **Dashboard HTML** único con todas las visualizaciones

## Uso Básico

```bash
# Procesar última simulación
python process_simulation_logs.py

# Procesar simulación específica
python process_simulation_logs.py -s uav_control_20251124_025703_20251124_025703

# Modo verbose (debug)
python process_simulation_logs.py -v

# Con offset temporal
python process_simulation_logs.py --offset 2.5
```

## Configuración

### 1. FILES_TO_PROCESS

Define qué archivos procesar, en qué orden y cómo mostrarlos:

```python
FILES_TO_PROCESS = [
    # (código, título, es_parámetro)
    ('VCGA', 'Ganancias Control de Velocidad', True),   # Mostrar como texto
    ('EKFS', 'Estado EKF', False),                      # Graficar
    ('GTPR', 'Posición Ground Truth', False),
]
```

**Parámetros:**
- `código`: Código de 4 letras del archivo (ej: 'EKFS', 'GTPR')
- `título`: Título descriptivo para mostrar en el dashboard
- `es_parámetro`: 
  - `True` = Mostrar como archivo de texto formateado
  - `False` = Generar gráfica interactiva

**Características:**
- Solo se procesan archivos definidos en esta lista
- El orden en el dashboard respeta el orden de la lista
- Los códigos deben coincidir con los archivos CSV generados

### 2. COMPARISON_GRAPHS

Define gráficas de comparación entre múltiples archivos:

```python
COMPARISON_GRAPHS = [
    {
        'title': 'Comparación Posición: EKF vs Ground Truth',
        'comparisons': [
            {
                'file1': 'EKFS',      # Primer archivo
                'field1': 'PosX',     # Campo del primer archivo
                'file2': 'GTPR',      # Segundo archivo
                'field2': 'Pr_x',     # Campo del segundo archivo
                'label': 'Posición X' # Etiqueta descriptiva
            },
            {
                'file1': 'EKFS',
                'field1': 'PosY',
                'file2': 'GTPR',
                'field2': 'Pr_y',
                'label': 'Posición Y'
            },
            # ... más comparaciones
        ],
        'show_error': True,  # Mostrar error en subplot separado
        'error_label': 'Error Posición [m]'
    },
]
```

**Características:**
- Compara múltiples señales en una sola gráfica
- Calcula y muestra el error automáticamente (si `show_error=True`)
- Interpola datos para alinear temporalmente las señales
- Usa líneas sólidas para señal 1 y líneas punteadas para señal 2
- Se agregan al dashboard al final (después de gráficas individuales)

## Estructura del Output

```
reports/
└── uav_control_YYYYMMDD_HHMMSS/
    ├── dashboard_*.html           # Dashboard principal
    ├── vcga_VCGA.txt             # Parámetros formateados
    ├── cvga_CVGA.txt
    └── graficas/
        ├── EKFS_EKFS.html        # Gráficas individuales
        ├── GTPR_GTPR.html
        ├── COMP_1_*.html         # Gráficas de comparación
        └── COMP_2_*.html
```

## Ejemplos de Uso

### Ejemplo 1: Agregar Nuevo Archivo para Graficar

```python
FILES_TO_PROCESS = [
    # ... archivos existentes ...
    ('NEWF', 'Mi Nuevo Archivo', False),  # Agregar al final
]
```

### Ejemplo 2: Agregar Archivo de Parámetros

```python
FILES_TO_PROCESS = [
    ('CONF', 'Configuración del Sistema', True),  # Mostrar como texto
    # ...
]
```

### Ejemplo 3: Comparar Aceleraciones

```python
COMPARISON_GRAPHS = [
    {
        'title': 'Comparación Aceleración: EKF vs Ground Truth',
        'comparisons': [
            {'file1': 'EKFS', 'field1': 'AccX', 'file2': 'GTAC', 'field2': 'Ax', 'label': 'Aceleración X'},
            {'file1': 'EKFS', 'field1': 'AccY', 'file2': 'GTAC', 'field2': 'Ay', 'label': 'Aceleración Y'},
            {'file1': 'EKFS', 'field1': 'AccZ', 'file2': 'GTAC', 'field2': 'Az', 'label': 'Aceleración Z'},
        ],
        'show_error': True,
        'error_label': 'Error Aceleración [m/s²]'
    },
]
```

### Ejemplo 4: Comparación sin Error

```python
COMPARISON_GRAPHS = [
    {
        'title': 'Velocidad Deseada vs Real',
        'comparisons': [
            {'file1': 'CVDS', 'field1': 'VdX', 'file2': 'EKFS', 'field2': 'VelX', 'label': 'Velocidad X'},
        ],
        'show_error': False,  # No mostrar subplot de error
    },
]
```

## Formato de los Archivos CSV

Los archivos CSV deben tener:
- Primera columna: `TimeUS` (timestamp en microsegundos)
- Columnas siguientes: Datos numéricos

Ejemplo:
```csv
TimeUS,PosX,PosY,PosZ
0,1.5,2.3,4.1
100000,1.6,2.4,4.2
```

## Argumentos de Línea de Comandos

```
-s, --simulation NAME   Nombre de la simulación a procesar
-r, --root PATH         Directorio raíz de simulaciones
-o, --offset SECONDS    Offset temporal en segundos
--individual            Crear archivos HTML individuales (sin dashboard)
-v, --verbose           Modo debug con información detallada
```

## Características Técnicas

### Interpolación de Datos
Para las gráficas de comparación, el sistema:
- Interpola automáticamente los datos del segundo archivo a los tiempos del primero
- Usa interpolación de vecino más cercano
- Tolerancia de 0.1 segundos
- Permite comparar archivos con diferentes frecuencias de muestreo

### Gestión de Errores
- Verifica existencia de archivos antes de procesar
- Valida campos/columnas en los CSV
- Reporta archivos o campos faltantes
- Continúa procesando incluso si algunas gráficas fallan

### Optimizaciones
- Solo procesa archivos definidos en `FILES_TO_PROCESS`
- Reutiliza DataFrames cargados para comparaciones múltiples
- Genera gráficas individuales y dashboard en paralelo
- Flush automático para evitar pérdida de datos

## Solución de Problemas

### "⊗ CÓDIGO - Título (No encontrado)"
- El archivo CSV no existe en la carpeta de simulación
- Verifica que el código de 4 letras coincida con el nombre del archivo

### "⚠ Campo 'campo' no encontrado en CÓDIGO"
- El campo especificado no existe en el CSV
- Usa `head -1 archivo.csv` para ver los campos disponibles

### "⚠ No se generaron gráficas"
- Ningún archivo de `FILES_TO_PROCESS` fue encontrado
- Verifica la ruta de la simulación

### Archivos de parámetros no se muestran
- Asegúrate de que el Logger esté inicializado antes de escribir
- Verifica que el formato de logging sea correcto
- Los logs deben escribirse en `initialize()`, no en constructores

## Extensiones Futuras

Posibles mejoras:
- [ ] Métricas estadísticas automáticas (RMSE, MAE)
- [ ] Gráficas 3D para trayectorias
- [ ] Exportación a PDF
- [ ] Análisis de frecuencia (FFT)
- [ ] Detección automática de anomalías
- [ ] Comparación entre múltiples simulaciones
