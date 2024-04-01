import plotly.graph_objects as go

def plot(x, y, xlabel=None, ylabel=None, title=None, trace_names=None):
    """
    Generate a Plotly plot.

    Args:
        x: List or array-like, x values.
        y: Array-like or list of array-like, y values.
        xlabel: (Optional) String, label for the x-axis. Default is None.
        ylabel: (Optional) String, label for the y-axis. Default is None.
        title: (Optional) String, title of the plot. Default is None.
        trace_names: (Optional) List of strings, names for each trace. Default is None.

    Returns:
        None (displays the plot).
    """

    # Convert y to a list of arrays if it's not already
    if not isinstance(y, list):
        y = [y]

    # Create traces
    traces = []
    if trace_names is None:
        trace_names = ['Trace {}'.format(i+1) for i in range(len(y))]

    for i, name in enumerate(trace_names):
        trace = go.Scatter(x=x, y=y[i], mode='lines', name=name)
        traces.append(trace)

    # Create layout
    layout = go.Layout(title=title, xaxis=dict(title=xlabel), yaxis=dict(title=ylabel))

    # Create figure
    fig = go.Figure(data=traces, layout=layout)

    # Display the figure
    fig.show()