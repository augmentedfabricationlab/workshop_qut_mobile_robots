from compas.geometry import Frame
from compas.geometry import Transformation

from assembly_information_model import Part

class MarkedPart(Part):
    @property
    def __data__(self):
        return {
            "attributes": self.attributes,
            "key": self.key,
            "frame": self.frame.__data__,
            "marker_frame": self.marker_frame.__data__ if self.marker_frame else None,
            "marker_id": self.marker_id
        }

    @classmethod
    def __from_data__(cls, data):
        part = cls()
        part.attributes.update(data["attributes"] or {})
        part.key = data["key"]
        part.frame = Frame.__from_data__(data["frame"])
        if data["marker_frame"]:
            part.marker_frame = Frame.__from_data__(data["marker_frame"])
        part.marker_id = data["marker_id"]
        return part

    def __init__(self, name=None, frame=None, **kwargs):
        super(Part, self).__init__()
        self.attributes = {"name": name or "Part"}
        self.attributes.update(kwargs)
        self.key = None
        self.frame = frame or Frame.worldXY()
        self.marker_frame = None  # Frame of the marker on the part
        self.marker_id = None     # ID of the marker on the part

    def transform(self, T):
        """Transforms the element.

        Parameters
        ----------
        T : :class:`Transformation`

        Returns
        -------
        None

        Examples
        --------
        >>> from compas.geometry import Box
        >>> from compas.geometry import Translation
        >>> part = Part.from_shape(Box(Frame.worldXY(), 1, 1, 1))
        >>> part.transform(Translation.from_vector([1, 0, 0]))
        """
        self.frame.transform(T)
        if self.marker_frame:
            self.marker_frame.transform(T)

        if 'mesh' in self.attributes.keys():
            self.attributes['mesh'].transform(T)

        if 'shape' in self.attributes.keys():
            self.attributes['shape'].transform(T)

    def transformed(self, T):
        """Returns a transformed copy of this part.

        Parameters
        ----------
        T : :class:`Transformation`

        Returns
        -------
        Part

        Examples
        --------
        >>> from compas.geometry import Box
        >>> from compas.geometry import Translation
        >>> part = Part.from_shape(Box(Frame.worldXY(), 1, 1, 1))
        >>> part2 = part.transformed(Translation.from_vector([1, 0, 0]))
        """
        part = self.copy()
        part.transform(T)
        return part

    def copy(self):
        """Returns a copy of this part.

        Returns
        -------
        Part
        """
        part = Part(name=self.attributes['name'], frame=self.frame.copy())
        # part.attributes.update(self.attributes)
        part.key = self.key
        part.marker_frame = self.marker_frame.copy() if self.marker_frame else None
        part.marker_id = self.marker_id
        
        if 'mesh' in self.attributes.keys():
            part.attributes.update({'mesh':self.attributes['mesh'].copy()})
        if 'shape' in self.attributes.keys():
            part.attributes.update({'shape':self.attributes['shape'].copy()})

        return part