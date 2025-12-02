import ComponentTypes from '@theme-original/NavbarItem/ComponentTypes';
import NavbarAuth from '@/components/NavbarAuth';
import { LanguageToggle } from '@/components/LanguageToggle';
import SearchBarNavbarItem from './SearchBar';

export default {
  ...ComponentTypes,
  'custom-navbarAuth': NavbarAuth,
  'custom-languageToggle': LanguageToggle,
  'custom-searchBar': SearchBarNavbarItem,
};
