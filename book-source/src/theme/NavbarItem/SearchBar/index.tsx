import React from 'react';
import { SearchBar as CustomSearchBar } from '@/components/SearchBar';

/**
 * Custom Search Bar Navbar Item
 *
 * This integrates the search bar into the Docusaurus navbar
 */
export default function SearchBarNavbarItem(): React.ReactElement {
  return <CustomSearchBar />;
}
