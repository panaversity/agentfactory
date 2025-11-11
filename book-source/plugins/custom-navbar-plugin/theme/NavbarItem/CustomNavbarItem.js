import React from 'react';
import Translations from '@site/src/components/Translations';

function CustomNavbarItem() {
  return (
    <div style={{position: 'absolute', right: '100px', top: '12px'}}>
      <Translations />
    </div>
  );
}

export default CustomNavbarItem;
