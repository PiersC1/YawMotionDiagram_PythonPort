from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, Literal
from abc import ABC, abstractmethod

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from numpy.typing import ArrayLike

Color = str | tuple | None # Type aliases

@dataclass
class PlotElement(ABC):
    """Base class for all plot elements."""
    x: ArrayLike
    y: ArrayLike
    label: str | None = None
    alpha: float = 1.0
    zorder: int | None = None
    
    def __post_init__(self):
        self.x = np.asarray(self.x)
        self.y = np.asarray(self.y)
    
    @abstractmethod
    def render(self, ax: Axes) -> Any:
        """Draw this element on the axes."""
        pass
    
    def get_mappable(self) -> ScalarMappable | None:
        """Return mappable for colorbar (if applicable)."""
        return None


@dataclass
class Line(PlotElement):
    """
    A line plot element.
    
    Args:
        x, y: Data arrays
        label: Legend label
        color: Line color
        linestyle: "-", "--", "-.", ":"
        linewidth: Line width
        marker: Marker style (None, "o", "s", "^", etc.)
        markersize: Marker size
    """
    color: Color = None
    linestyle: str = "-"
    linewidth: float = 1.5
    marker: str | None = None
    markersize: float = 6.0
    markevery: int | None = None
    
    def render(self, ax: Axes) -> Any:
        kwargs = dict(
            color=self.color,
            linestyle=self.linestyle,
            linewidth=self.linewidth,
            marker=self.marker,
            markersize=self.markersize,
            alpha=self.alpha,
            label=self.label,
        )
        if self.zorder is not None:
            kwargs["zorder"] = self.zorder
        if self.markevery is not None:
            kwargs["markevery"] = self.markevery
        
        line, = ax.plot(self.x, self.y, **kwargs)
        return line


@dataclass
class Scatter(PlotElement):
    """
    A scatter plot element.
    
    Args:
        x, y: Data arrays
        c: Color data array (for colormap) or single color
        s: Size (single value or array)
        cmap: Colormap name
        vmin, vmax: Color normalization limits
        marker: Marker style
    """
    c: ArrayLike | Color = None
    s: ArrayLike | float = 36.0
    cmap: str = "viridis"
    vmin: float | None = None
    vmax: float | None = None
    marker: str = "o"
    edgecolors: str = "face"
    linewidths: float = 0.5
    
    _mappable: ScalarMappable | None = field(default=None, init=False, repr=False)
    
    def render(self, ax: Axes) -> Any:
        kwargs = dict(
            s=self.s,
            marker=self.marker,
            edgecolors=self.edgecolors,
            linewidths=self.linewidths,
            alpha=self.alpha,
            label=self.label,
        )
        if self.zorder is not None:
            kwargs["zorder"] = self.zorder
        
        # Check if c is color data (array same length as x)
        has_colordata = False
        if self.c is not None:
            try:
                c_arr = np.asarray(self.c)
                if c_arr.ndim == 1 and len(c_arr) == len(self.x):
                    has_colordata = True
            except (ValueError, TypeError):
                pass
        
        if has_colordata:
            kwargs["c"] = self.c
            kwargs["cmap"] = self.cmap
            kwargs["vmin"] = self.vmin
            kwargs["vmax"] = self.vmax
        elif self.c is not None:
            kwargs["c"] = self.c
        
        scatter = ax.scatter(self.x, self.y, **kwargs)
        
        if has_colordata:
            self._mappable = scatter
        
        return scatter
    
    def get_mappable(self) -> ScalarMappable | None:
        return self._mappable


@dataclass
class ColorLine(PlotElement):
    """
    A line with color mapped to a third variable.
    
    Args:
        x, y: Data arrays
        c: Color data array (same length as x, y)
        cmap: Colormap name
        vmin, vmax: Color normalization limits
        linewidth: Line width
    """
    c: ArrayLike = None
    cmap: str = "viridis"
    vmin: float | None = None
    vmax: float | None = None
    linewidth: float = 2.0
    
    _mappable: ScalarMappable | None = field(default=None, init=False, repr=False)
    
    def __post_init__(self):
        super().__post_init__()
        if self.c is None:
            self.c = np.arange(len(self.x))
        else:
            self.c = np.asarray(self.c)
    
    def render(self, ax: Axes) -> LineCollection:
        # Create line segments
        points = np.array([self.x, self.y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        
        # Color normalization
        vmin = self.vmin if self.vmin is not None else np.nanmin(self.c)
        vmax = self.vmax if self.vmax is not None else np.nanmax(self.c)
        norm = Normalize(vmin=vmin, vmax=vmax)
        
        lc = LineCollection(
            segments,
            cmap=self.cmap,
            norm=norm,
            linewidth=self.linewidth,
            alpha=self.alpha,
        )
        
        # Use midpoint colors for segments
        segment_colors = (self.c[:-1] + self.c[1:]) / 2
        lc.set_array(segment_colors)
        
        if self.zorder is not None:
            lc.set_zorder(self.zorder)
        if self.label is not None:
            lc.set_label(self.label)
        
        ax.add_collection(lc)
        ax.autoscale_view()
        
        self._mappable = lc
        return lc
    
    def get_mappable(self) -> ScalarMappable | None:
        return self._mappable


@dataclass
class Fill(PlotElement):
    """
    A filled region.
    
    Args:
        x, y: Data arrays (fills between y and 0, or y and y2)
        y2: Second y boundary (optional)
        color: Fill color
    """
    y2: ArrayLike | float | None = None
    color: Color = None
    edgecolor: Color = None
    
    def render(self, ax: Axes) -> Any:
        kwargs = dict(
            facecolor=self.color,
            edgecolor=self.edgecolor,
            alpha=self.alpha,
            label=self.label,
        )
        if self.zorder is not None:
            kwargs["zorder"] = self.zorder
        
        if self.y2 is None:
            return ax.fill_between(self.x, self.y, **kwargs)
        else:
            return ax.fill_between(self.x, self.y, self.y2, **kwargs)


@dataclass
class ErrorBar(PlotElement):
    """
    Line plot with error bars.
    
    Args:
        x, y: Data arrays
        yerr: Y error (symmetric or [lower, upper])
        xerr: X error (optional)
    """
    yerr: ArrayLike | None = None
    xerr: ArrayLike | None = None
    color: Color = None
    linestyle: str = "-"
    linewidth: float = 1.5
    marker: str = "o"
    markersize: float = 6.0
    capsize: float = 3.0
    
    def render(self, ax: Axes) -> Any:
        kwargs = dict(
            yerr=self.yerr,
            xerr=self.xerr,
            color=self.color,
            linestyle=self.linestyle,
            linewidth=self.linewidth,
            marker=self.marker,
            markersize=self.markersize,
            capsize=self.capsize,
            alpha=self.alpha,
            label=self.label,
        )
        if self.zorder is not None:
            kwargs["zorder"] = self.zorder
        
        return ax.errorbar(self.x, self.y, **kwargs)


class Figure:
    """
    Main figure container.
    
    Example:
        fig = Figure(nrows=2, ncols=1, figsize=(10, 8))
        fig.add(Line(x, y1), subplot=0)
        fig.add(Line(x, y2), subplot=1)
        fig.set_labels(subplot=0, xlabel="X", ylabel="Y1")
        fig.set_labels(subplot=1, xlabel="X", ylabel="Y2")
        fig.render()
        fig.save("output.png")
    """
    
    def __init__(
        self,
        nrows: int = 1,
        ncols: int = 1,
        figsize: tuple[float, float] = (10, 6),
        dpi: float = 100,
        sharex: bool = False,
        sharey: bool = False,
    ):
        self.nrows = nrows
        self.ncols = ncols
        self.figsize = figsize
        self.dpi = dpi
        self.sharex = sharex
        self.sharey = sharey
        
        self._elements: dict[int, list[tuple[PlotElement, dict]]] = {
            i: [] for i in range(nrows * ncols)
        }
        self._config: dict[int, dict] = {i: {} for i in range(nrows * ncols)}
        self._fig = None
        self._axes = []
        self._suptitle = None
    
    def add(
        self,
        element: PlotElement,
        subplot: int = 0,
        colorbar: bool = False,
        cbar_label: str | None = None,
    ) -> "Figure":
        """Add a plot element to a subplot."""
        self._elements[subplot].append((element, {
            "colorbar": colorbar,
            "cbar_label": cbar_label,
        }))
        return self
    
    def set_labels(
        self,
        subplot: int = 0,
        xlabel: str | None = None,
        ylabel: str | None = None,
        title: str | None = None,
    ) -> "Figure":
        """Set axis labels and title for a subplot."""
        cfg = self._config[subplot]
        if xlabel is not None:
            cfg["xlabel"] = xlabel
        if ylabel is not None:
            cfg["ylabel"] = ylabel
        if title is not None:
            cfg["title"] = title
        return self
    
    def set_limits(
        self,
        subplot: int = 0,
        xlim: tuple | None = None,
        ylim: tuple | None = None,
    ) -> "Figure":
        """Set axis limits for a subplot."""
        cfg = self._config[subplot]
        if xlim is not None:
            cfg["xlim"] = xlim
        if ylim is not None:
            cfg["ylim"] = ylim
        return self
    
    def set_scale(
        self,
        subplot: int = 0,
        xscale: str = "linear",
        yscale: str = "linear",
    ) -> "Figure":
        """Set axis scale (linear, log, symlog)."""
        cfg = self._config[subplot]
        cfg["xscale"] = xscale
        cfg["yscale"] = yscale
        return self
    
    def legend(self, subplot: int = 0, loc: str = "best", **kwargs) -> "Figure":
        """Enable legend for a subplot."""
        self._config[subplot]["legend"] = {"loc": loc, **kwargs}
        return self
    
    def grid(self, subplot: int = 0, alpha: float = 0.3, **kwargs) -> "Figure":
        """Enable grid for a subplot."""
        self._config[subplot]["grid"] = {"alpha": alpha, **kwargs}
        return self
    
    def set_aspect(self, subplot: int = 0, aspect: str | float = "equal") -> "Figure":
        """Set aspect ratio for a subplot."""
        self._config[subplot]["aspect"] = aspect
        return self
    
    def suptitle(self, title: str, **kwargs) -> "Figure":
        """Set figure super title."""
        self._suptitle = (title, kwargs)
        return self
    
    def get_ax(self, subplot: int = 0) -> Axes | None:
        """Get the matplotlib Axes for a subplot (after render)."""
        if subplot < len(self._axes):
            return self._axes[subplot]
        return None
    
    def render(self) -> "Figure":
        """Render the figure."""
        self._fig, axes = plt.subplots(
            self.nrows, self.ncols,
            figsize=self.figsize,
            dpi=self.dpi,
            sharex=self.sharex,
            sharey=self.sharey,
            squeeze=False,
            constrained_layout=True,
        )
        self._axes = axes.flatten().tolist()
        
        # Render elements
        for subplot_idx, elements in self._elements.items():
            ax = self._axes[subplot_idx]
            
            for element, opts in elements:
                element.render(ax)
                
                # Add colorbar if requested
                if opts.get("colorbar"):
                    mappable = element.get_mappable()
                    if mappable is not None:
                        cbar = self._fig.colorbar(mappable, ax=ax)
                        if opts.get("cbar_label"):
                            cbar.set_label(opts["cbar_label"])
            
            # Apply config
            cfg = self._config[subplot_idx]
            if "xlabel" in cfg:
                ax.set_xlabel(cfg["xlabel"])
            if "ylabel" in cfg:
                ax.set_ylabel(cfg["ylabel"])
            if "title" in cfg:
                ax.set_title(cfg["title"])
            if "xlim" in cfg:
                ax.set_xlim(cfg["xlim"])
            if "ylim" in cfg:
                ax.set_ylim(cfg["ylim"])
            if "xscale" in cfg:
                ax.set_xscale(cfg["xscale"])
            if "yscale" in cfg:
                ax.set_yscale(cfg["yscale"])
            if "legend" in cfg:
                ax.legend(**cfg["legend"])
            if "grid" in cfg:
                ax.grid(True, **cfg["grid"])
            if "aspect" in cfg:
                ax.set_aspect(cfg["aspect"])
        
        if self._suptitle:
            self._fig.suptitle(self._suptitle[0], **self._suptitle[1])
        
        return self
    
    def save(self, filename: str, dpi: float | None = None, **kwargs) -> "Figure":
        """Save the figure to a file."""
        if self._fig is None:
            self.render()
        self._fig.savefig(filename, dpi=dpi or self.dpi, bbox_inches="tight", **kwargs)
        return self
    
    def show(self) -> "Figure":
        """Display the figure."""
        if self._fig is None:
            self.render()
        plt.show()
        return self
    
    def close(self):
        """Close the figure."""
        if self._fig is not None:
            plt.close(self._fig)
            self._fig = None
    
    def __enter__(self):
        return self
    
    def __exit__(self, *args):
        self.close()


# additional functions, can probably imporve later
def quick_plot(x, y, **kwargs) -> Figure:
    """Quick single line plot."""
    fig = Figure()
    fig.add(Line(x, y, **kwargs))
    return fig


def quick_scatter(x, y, **kwargs) -> Figure:
    """Quick scatter plot."""
    fig = Figure()
    fig.add(Scatter(x, y, **kwargs))
    return fig