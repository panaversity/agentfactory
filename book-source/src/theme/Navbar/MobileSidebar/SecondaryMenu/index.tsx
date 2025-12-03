import React from 'react';
import { useNavbarSecondaryMenu } from '@docusaurus/theme-common/internal';

// Custom SecondaryMenu without the "Back to main menu" button
// Back button is now in the header component
export default function NavbarMobileSidebarSecondaryMenu(): React.ReactElement | null {
  const secondaryMenu = useNavbarSecondaryMenu();

  // Just render the secondary menu content without the back button
  return <>{secondaryMenu.content}</>;
}
