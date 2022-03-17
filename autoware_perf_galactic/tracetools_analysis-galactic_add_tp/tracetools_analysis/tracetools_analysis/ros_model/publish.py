import collections.abc
# from .callback import Callback

class Publish():
    def __init__(self, topic_name, callback, object=None):
        assert isinstance(topic_name, str)
        # if callback is not None:
        #     assert isinstance(callback, Callback)
        self.topic_name = topic_name
        self.object = object
        self.callback = callback

    def has_callback(self, callback):
        return callback == self.callback

class PublishCollection(collections.abc.Iterable):
    def __init__(self):
        self._pubs = []

    def __iter__(self):
        return PublishCollectionIterator(self)

    def __len__(self):
        return len(self._pubs)

    def __getitem__(self, key):
        return self._pubs[key]

    def get_callbacks(self, topic_name):
        callbacks = [pub.callback for pub in self._pubs if pub.topic_name == topic_name]
        return callbacks

    def callbacks(self):
        return [pub.callback for pub in self._pubs]

    def get_info(self):
        pub_list = {}
        for pub in self._pubs:
            if pub.callback is not None:
                pub_list[pub.topic_name] = pub.callback.symbol
            else:
                pub_list[pub.topic_name] = ''

        if len(pub_list) == 0:
            pub_list[''] = ''

        return pub_list

    def append(self, pub):
        assert isinstance(pub, Publish), pub
        self._pubs.append(pub)


class PublishCollectionIterator(collections.abc.Iterator):
    def __init__(self, pub_collection):
        self._i = 0
        self._pub_collection = pub_collection

    def __next__(self):
        try:
            v = self._pub_collection._pubs[self._i]
            self._i += 1
            return v
        except IndexError:
            raise StopIteration
