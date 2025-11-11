declare global {
  namespace google {
    namespace translate {
      class TranslateElement {
        constructor(options: object, element: string);
        InlineLayout: { SIMPLE: number };
      }
    }
  }

  interface Window {
    google: typeof google;
    googleTranslateElementInit: () => void;
  }
}

export {};
