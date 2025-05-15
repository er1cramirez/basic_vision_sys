import pandas as pd
import matplotlib.pyplot as plt
plt.style.use('ggplot')
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.express as px

app = dash.Dash(__name__)
df_global = None

app.layout = html.Div([
    dcc.Upload(
        id='upload-data',
        children=html.Div(['Drag and Drop or ', html.A('Select a CSV File')]),
        style={
            'width': '100%',
            'height': '60px',
            'lineHeight': '60px',
            'borderWidth': '1px',
            'borderStyle': 'dashed',
            'borderRadius': '5px',
            'textAlign': 'center',
            'margin': '10px'
        }
    ),
    dcc.Dropdown(id='column-selector', multi=True),
    dcc.Graph(id='csv-graph')
])

@app.callback(
    [Output('column-selector', 'options'),
     Output('column-selector', 'value')],
    Input('upload-data', 'contents'),
    prevent_initial_call=True
)
def update_dropdown(contents):
    global df_global
    if contents is not None:
        import io, base64
        content_type, content_string = contents.split(',')
        decoded = base64.b64decode(content_string)
        df_global = pd.read_csv(io.StringIO(decoded.decode('utf-8')))
        df_global.iloc[:,0] = (df_global.iloc[:,0] - df_global.iloc[0,0]) / 1_000_000.0
        columns = list(df_global.columns)[1:]
        return ([{'label': c, 'value': c} for c in columns], columns)
    return ([], [])

@app.callback(
    Output('csv-graph', 'figure'),
    Input('column-selector', 'value')
)
def update_graph(selected_columns):
    if df_global is not None and selected_columns:
        fig = px.line(
            df_global,
            x=df_global.columns[0],
            y=selected_columns,
            template="ggplot2",
            title="Log Data Visualization"
        )
        fig.update_layout(xaxis_title="Time (s)", yaxis_title="Value")
        return fig
    return {}

def main():
    app.run(debug=False)

if __name__ == "__main__":
    main()