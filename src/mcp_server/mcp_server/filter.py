# Copyright 2024 Nicolas Gres
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from fnmatch import fnmatch
from typing import Sequence
from mcp_server.subject.subject import Subject


class EntityFilter:
    """
    A filter for entities based on their type and name.
    """

    def __init__(
        self, include_list: Sequence[str] = [], exclude_list: Sequence[str] = []
    ):
        self.include_list = include_list
        self.exclude_list = exclude_list

    def matches(self, entity: Subject) -> bool:
        """
        Check if the entity name matches the include and exclude lists.
        """
        return self._match_one(entity.name, self.include_list) and not self._match_one(
            entity.name, self.exclude_list
        )

    def apply(self, entities: Sequence[Subject]) -> Sequence[Subject]:
        """
        Apply the filter to a list of entities.
        Returns a list of entities that match the filter criteria.
        """
        return [entity for entity in entities if self.matches(entity)]

    def _match_one(self, name, list):
        for pat in list:
            if fnmatch(name, pat):
                return True
        return False
